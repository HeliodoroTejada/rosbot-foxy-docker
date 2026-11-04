"""Exercise the installed converter with synthetic best-effort camera data."""
from collections import deque
import math
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


DEPTH_TOPIC = '/astra_cloud_test/depth/image_raw'
INFO_TOPIC = '/astra_cloud_test/depth/camera_info'
CLOUD_TOPIC = '/astra_cloud_test/points'


@pytest.fixture(scope='module', params=[(1, 0.0), (2, 5.0)],
                ids=['full_unlimited', 'sampled_5hz'])
def converter(tmp_path_factory, request):
    pixel_stride, max_rate = request.param
    rclpy.init()
    node = rclpy.create_node('astra_cloud_test_source')
    images = node.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
    infos = node.create_publisher(CameraInfo, INFO_TOPIC, qos_profile_sensor_data)
    clouds = deque(maxlen=200)
    subscription = node.create_subscription(
        PointCloud2, CLOUD_TOPIC, clouds.append, qos_profile_sensor_data)
    log_path = tmp_path_factory.mktemp('astra_cloud') / 'converter.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'astra_camera', 'depth_to_cloud.launch.py',
            'depth_topic:=' + DEPTH_TOPIC,
            'camera_info_topic:=' + INFO_TOPIC,
            'cloud_topic:=' + CLOUD_TOPIC,
            'pixel_stride:=' + str(pixel_stride),
            'max_rate:=' + str(max_rate),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                rclpy.spin_once(node, timeout_sec=0.05)
                if (node.count_subscribers(DEPTH_TOPIC)
                        and node.count_subscribers(INFO_TOPIC)
                        and node.count_publishers(CLOUD_TOPIC)):
                    break
            else:
                pytest.fail('Converter topics not discovered:\n' + log_path.read_text())
            yield node, images, infos, clouds, log_path, pixel_stride, max_rate
        finally:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@pytest.mark.parametrize('encoding,fmt,depths', [
    ('32FC1', 'f', [1.0, 2.0, float('nan'), 0.5, 1.5,
                    3.0, 1.0, 2.0, 0.6, 0.7, 1.5, 2.5, 3.0, 4.0, 5.0]),
    ('16UC1', 'H', [1000, 2000, 0, 500, 1500,
                    3000, 1000, 2000, 600, 700, 1500, 2500, 3000, 4000, 5000]),
])
def test_xyz_units_invalid_depth_and_optical_frame(converter, encoding, fmt, depths):
    node, images, infos, clouds, log_path, stride, _ = converter
    clouds.clear()
    first_stamp = node.get_clock().now().nanoseconds
    deadline = time.monotonic() + 8.0
    result = None
    sent_stamps = set()
    while time.monotonic() < deadline:
        image = Image()
        image.header.stamp = node.get_clock().now().to_msg()
        sent_stamps.add(image.header.stamp.sec * 1000000000 + image.header.stamp.nanosec)
        image.header.frame_id = 'openni_depth_optical_frame'
        image.width, image.height = 5, 3
        image.encoding = encoding
        image.is_bigendian = False
        # Include padding so sampling must respect the input row stride.
        image.step = (image.width + 2) * struct.calcsize(fmt)
        image.data = b''.join(
            struct.pack('<' + fmt * image.width, *depths[v * 5:(v + 1) * 5])
            + b'\xff' * (2 * struct.calcsize(fmt)) for v in range(image.height))

        info = CameraInfo()
        info.header = image.header
        info.width, info.height = image.width, image.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [100.0, 0.0, 1.0, 0.0, 200.0, 0.5, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [100.0, 0.0, 1.0, 0.0, 0.0, 200.0, 0.5, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        infos.publish(info)
        images.publish(image)
        rclpy.spin_once(node, timeout_sec=0.05)
        for cloud in clouds:
            stamp = cloud.header.stamp.sec * 1000000000 + cloud.header.stamp.nanosec
            if stamp >= first_stamp:
                result = cloud
                break
        if result is not None:
            break
    assert result is not None, 'No cloud from best-effort inputs:\n' + log_path.read_text()
    assert (result.width, result.height) == ((5 + stride - 1) // stride,
                                           (3 + stride - 1) // stride)
    assert result.header.frame_id == 'openni_depth_optical_frame'
    assert result.header.stamp.sec * 1000000000 + result.header.stamp.nanosec in sent_stamps
    assert not result.is_dense
    offsets = {field.name: field.offset for field in result.fields}
    endian = '>' if result.is_bigendian else '<'
    expected_depths = [1.0, 2.0, float('nan'), 0.5, 1.5,
                       3.0, 1.0, 2.0, 0.6, 0.7, 1.5, 2.5, 3.0, 4.0, 5.0]
    for index in range(result.width * result.height):
        u, v = index % result.width, index // result.width
        source_u, source_v = u * stride, v * stride
        z = expected_depths[source_v * 5 + source_u]
        start = v * result.row_step + u * result.point_step
        xyz = [struct.unpack_from(endian + 'f', result.data, start + offsets[k])[0]
               for k in ('x', 'y', 'z')]
        if not math.isfinite(z):
            assert all(math.isnan(value) for value in xyz)
        else:
            assert xyz == pytest.approx(((source_u - 1.0) * z / 100.0,
                                         (source_v - 0.5) * z / 200.0, z), abs=1e-6)


def test_bad_calibration_and_frame_mismatch_do_not_produce_clouds(converter):
    node, images, infos, clouds, log_path, _, _ = converter

    def publish_pair(problem=None):
        image = Image()
        image.header.stamp = node.get_clock().now().to_msg()
        image.header.frame_id = 'openni_depth_optical_frame'
        image.width = image.height = 1
        image.encoding = '32FC1'
        image.step = 4
        image.data = struct.pack('<f', 1.0)
        info = CameraInfo()
        info.header = image.header
        info.width = info.height = 1
        info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.r = list(info.k)
        info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        if problem == 'calibration':
            info.p[0] = 0.0
        elif problem == 'frame':
            # Headers must be separate objects for this intentional mismatch.
            info.header = type(image.header)()
            info.header.stamp = image.header.stamp
            info.header.frame_id = 'different_optical_frame'
        infos.publish(info)
        images.publish(image)

    def newer_than(stamp):
        return [cloud for cloud in clouds
                if cloud.header.stamp.sec * 1000000000
                + cloud.header.stamp.nanosec >= stamp]

    for problem in ('calibration', 'frame'):
        clouds.clear()
        first_stamp = node.get_clock().now().nanoseconds
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            publish_pair(problem)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert not newer_than(first_stamp), 'Invalid inputs generated a cloud'

        # Ensure rejection did not crash or stall the converter.
        first_stamp = node.get_clock().now().nanoseconds
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not newer_than(first_stamp):
            publish_pair()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert newer_than(first_stamp), 'Converter did not recover:\n' + log_path.read_text()


def test_rate_limit_survives_sensor_timestamp_rewind(converter):
    node, images, infos, clouds, log_path, stride, max_rate = converter
    image = Image()
    image.header.frame_id = 'openni_depth_optical_frame'
    image.width, image.height = (640, 480) if max_rate else (1, 1)
    image.encoding = '32FC1'
    image.step = 4 * image.width
    image.data = struct.pack('<f', 1.0) * (image.width * image.height)
    info = CameraInfo()
    info.width, info.height = image.width, image.height
    info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.r = list(info.k)
    info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0]
    clouds.clear()
    sent = {}
    start = time.monotonic()
    duration = 2.0
    while time.monotonic() - start < duration:
        elapsed = time.monotonic() - start
        # A backwards sensor timestamp must not stall or bypass the steady-clock limit.
        stamp = node.get_clock().now().nanoseconds + (100000000000 if elapsed < 1.0 else 0)
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1000000000)
        info.header = image.header
        sent[stamp] = elapsed
        infos.publish(info)
        images.publish(image)
        rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.015)
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    selected = [sent[cloud.header.stamp.sec * 1000000000 + cloud.header.stamp.nanosec]
                for cloud in clouds
                if cloud.header.stamp.sec * 1000000000 + cloud.header.stamp.nanosec in sent]
    for cloud in clouds:
        if cloud.header.stamp.sec * 1000000000 + cloud.header.stamp.nanosec in sent:
            assert (cloud.width, cloud.height) == ((image.width + stride - 1) // stride,
                                                  (image.height + stride - 1) // stride)
            assert len(cloud.data) == cloud.width * cloud.height * 16
    assert len(selected) >= 3, log_path.read_text()
    assert min(selected) < 0.8 and max(selected) > 1.2, 'Converter stalled on timestamp rewind'
    if max_rate:
        assert len(selected) <= math.ceil(duration * max_rate) + 1
        assert all(b - a >= 0.9 / max_rate for a, b in zip(selected, selected[1:]))
    else:
        assert len(selected) > 12, 'Unlimited mode unexpectedly throttled the input'
