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


@pytest.fixture(scope='module')
def converter(tmp_path_factory):
    rclpy.init()
    node = rclpy.create_node('astra_cloud_test_source')
    images = node.create_publisher(Image, DEPTH_TOPIC, qos_profile_sensor_data)
    infos = node.create_publisher(CameraInfo, INFO_TOPIC, qos_profile_sensor_data)
    clouds = deque(maxlen=10)
    subscription = node.create_subscription(
        PointCloud2, CLOUD_TOPIC, clouds.append, qos_profile_sensor_data)
    log_path = tmp_path_factory.mktemp('astra_cloud') / 'converter.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'astra_camera', 'depth_to_cloud.launch.py',
            'depth_topic:=' + DEPTH_TOPIC,
            'camera_info_topic:=' + INFO_TOPIC,
            'cloud_topic:=' + CLOUD_TOPIC,
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
            yield node, images, infos, clouds, log_path
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
    ('32FC1', 'f', [1.0, 2.0, float('nan'), 0.5, 1.5, 3.0]),
    ('16UC1', 'H', [1000, 2000, 0, 500, 1500, 3000]),
])
def test_xyz_units_invalid_depth_and_optical_frame(converter, encoding, fmt, depths):
    node, images, infos, clouds, log_path = converter
    clouds.clear()
    first_stamp = node.get_clock().now().nanoseconds
    deadline = time.monotonic() + 8.0
    result = None
    while time.monotonic() < deadline:
        image = Image()
        image.header.stamp = node.get_clock().now().to_msg()
        image.header.frame_id = 'openni_depth_optical_frame'
        image.width, image.height = 3, 2
        image.encoding = encoding
        image.is_bigendian = False
        image.step = image.width * struct.calcsize(fmt)
        image.data = struct.pack('<' + fmt * len(depths), *depths)

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
    assert (result.width, result.height) == (3, 2)
    assert result.header.frame_id == 'openni_depth_optical_frame'
    assert not result.is_dense
    offsets = {field.name: field.offset for field in result.fields}
    endian = '>' if result.is_bigendian else '<'
    expected_depths = [1.0, 2.0, float('nan'), 0.5, 1.5, 3.0]
    for index, z in enumerate(expected_depths):
        u, v = index % 3, index // 3
        start = v * result.row_step + u * result.point_step
        xyz = [struct.unpack_from(endian + 'f', result.data, start + offsets[k])[0]
               for k in ('x', 'y', 'z')]
        if not math.isfinite(z):
            assert all(math.isnan(value) for value in xyz)
        else:
            assert xyz == pytest.approx(((u - 1.0) * z / 100.0,
                                         (v - 0.5) * z / 200.0, z), abs=1e-6)


def test_bad_calibration_and_frame_mismatch_do_not_produce_clouds(converter):
    node, images, infos, clouds, log_path = converter

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
