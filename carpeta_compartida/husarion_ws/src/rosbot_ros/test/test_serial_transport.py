"""Exercise wire framing and the ROS bridge through an isolated virtual UART."""
import io
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from serial_protocol import SerialFrameDecoder


def frame(topic, payload=b''):
    length = struct.pack('<H', len(payload))
    body = struct.pack('<H', topic) + payload
    return (b'\xff\xfe' + length + bytes([255 - sum(length) % 256])
            + body + bytes([255 - sum(body) % 256]))


def serialize(message):
    data = io.BytesIO()
    message.serialize(data)
    return data.getvalue()


@pytest.mark.parametrize('split', range(15))
def test_partial_frames_and_embedded_sync(split):
    decoder = SerialFrameDecoder()
    payload = b'ab\xff\xfecd'
    packet = frame(65530, payload)
    decoder.feed(packet[:split])
    result = decoder.next_packet(0.0)
    decoder.feed(packet[split:])
    result = result or decoder.next_packet(0.1)
    assert result == (65530, payload)
    assert decoder.next_packet(0.1) is None


def test_corrupt_frames_noise_and_timeout_recover():
    decoder = SerialFrameDecoder(timeout=0.1)
    bad_length = bytearray(frame(100, b'data'))
    bad_length[4] ^= 1
    bad_data = bytearray(frame(100, b'data'))
    bad_data[-1] ^= 1
    decoder.feed(b'noise\xff\xff' + bad_length + bad_data + frame(101, b'good'))
    assert decoder.next_packet(0.0) == (101, b'good')
    assert decoder.errors['length_checksum'] and decoder.errors['data_checksum']
    decoder.feed(frame(100, b'x' * 1000)[:8])
    assert decoder.next_packet(0.0) is None
    decoder.feed(frame(101, b'recovered'))
    assert decoder.next_packet(0.2) == (101, b'recovered')
    assert decoder.errors['timeout'] == 1


def test_unsigned_length_and_bounded_buffer():
    decoder = SerialFrameDecoder()
    payload = b'x' * 40000
    decoder.feed(frame(100, payload))
    assert decoder.next_packet(0.0) == (100, payload)
    decoder.feed(b'x' * (decoder.MAX_BUFFER + 10))
    assert len(decoder.buffer) <= decoder.MAX_BUFFER
    assert decoder.errors['overflow'] == 1
    decoder.feed(frame(102, b'good'))
    assert decoder.next_packet(0.0) == (102, b'good')


@pytest.fixture
def bridge():
    import rclpy
    from serialclient import SerialClient
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    os.set_blocking(master, False)
    rclpy.init(args=['--ros-args', '-p', 'port:=' + slave_name, '-p', 'baud:=115200'])
    node = None
    try:
        node = SerialClient(timeout=0.5)
        yield node, master, slave_name
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(master)
        os.close(slave)


def pose_info():
    from rosserial.geometry_msgs.msg import PoseStamped
    from rosserial.rosserial_msgs.msg import TopicInfo
    info = TopicInfo()
    info.topic_id = 100
    info.topic_name = '/serial_transport_test/pose'
    info.message_type = 'geometry_msgs/PoseStamped'
    info.md5sum = PoseStamped._md5sum
    info.buffer_size = 512
    return info


def test_renegotiation_reuses_publisher(bridge):
    node, _, _ = bridge
    info = pose_info()
    node.setupPublisher(serialize(info))
    first = node.serial_publishers[100]
    node.setupPublisher(serialize(info))
    assert node.serial_publishers[100] is first
    assert sum(p.topic_name == info.topic_name for p in node.publishers) == 1


def test_uart_is_exclusive_and_lock_is_released(bridge):
    from serial import Serial, SerialException
    node, _, slave_name = bridge
    assert node.port.timeout == 0
    with pytest.raises(SerialException):
        Serial(slave_name, 115200, timeout=0, exclusive=True)
    node.port.close()
    with Serial(slave_name, 115200, timeout=0, exclusive=True) as port:
        assert port.is_open


@pytest.mark.parametrize('packets,byte_budget,expected', [(3, 4096, 3), (64, 24, 2)])
def test_callback_budget_retains_unprocessed_data(bridge, packets, byte_budget, expected):
    node, master, _ = bridge
    node.read_timer.cancel()
    node.read_packet_budget = packets
    node.read_byte_budget = byte_budget
    node.read_time_budget = 0.1
    received = []
    node.callbacks[200] = received.append
    os.write(master, b''.join(frame(200, struct.pack('<I', i)) for i in range(10)))
    time.sleep(0.01)
    node.read_timer_callback()
    assert len(received) == expected
    for _ in range(10):
        node.read_timer_callback()
    assert [struct.unpack('<I', value)[0] for value in received] == list(range(10))


def test_partial_packet_does_not_block_other_callbacks(bridge):
    import rclpy
    node, master, _ = bridge
    ticks = []
    timer = node.create_timer(0.005, lambda: ticks.append(time.monotonic()))
    os.write(master, frame(100, b'incomplete')[:6])
    deadline = time.monotonic() + 0.08
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.002)
    assert len(ticks) >= 3
    node.destroy_timer(timer)


def test_time_budget_yields_between_packet_callbacks(bridge):
    node, master, _ = bridge
    node.read_timer.cancel()
    received = []

    def slow_callback(payload):
        received.append(payload)
        time.sleep(0.004)

    node.callbacks[200] = slow_callback
    os.write(master, frame(200, b'data') * 10)
    time.sleep(0.01)
    node.read_timer_callback()
    assert 1 <= len(received) <= 2
    for _ in range(10):
        node.read_timer_callback()
    assert len(received) == 10


def test_mixed_stream_over_100_frames_per_second_preserves_pose_time(bridge):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    from rosserial.geometry_msgs.msg import PoseStamped as WirePose
    from rosserial.rosserial_msgs.msg import TopicInfo
    node, master, _ = bridge
    received, other_frames, sent = [], [], {}
    node.callbacks[200] = other_frames.append

    def receive(message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        received.append((int(message.pose.position.x), stamp, time.monotonic()))

    subscription = node.create_subscription(
        PoseStamped, '/serial_transport_test/pose', receive, qos_profile_sensor_data)
    os.write(master, frame(TopicInfo.ID_PUBLISHER, serialize(pose_info())))
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.005)
        if 100 in node.serial_publishers and node.count_subscribers('/serial_transport_test/pose'):
            break
    else:
        pytest.fail('Virtual firmware publisher was not negotiated/discovered')

    start = time.monotonic()
    next_other = next_pose = next_sync = start
    count = 0
    while time.monotonic() - start < 2.0:
        now = time.monotonic()
        if now >= next_other:
            os.write(master, frame(200, b'background') * 2)
            count += 2
            next_other += 0.01
        if now >= next_pose:
            message = WirePose()
            # Intentionally old timestamps prove the bridge never hides age by restamping.
            stamp = node.get_clock().now().nanoseconds - 7000000000
            message.header.stamp.secs, message.header.stamp.nsecs = divmod(stamp, 1000000000)
            message.header.frame_id = 'odom'
            index = len(sent)
            message.pose.position.x = float(index)
            message.pose.orientation.w = 1.0
            sent[index] = (stamp, time.monotonic())
            os.write(master, frame(100, serialize(message)))
            next_pose += 0.05
        if now >= next_sync:
            os.write(master, frame(TopicInfo.ID_TIME))
            next_sync += 0.25
        rclpy.spin_once(node, timeout_sec=0.001)
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline and (len(received) < len(sent) or len(other_frames) < count):
        rclpy.spin_once(node, timeout_sec=0.002)
    assert count >= 300, 'Synthetic input must exceed the former 100-frame/s ceiling'
    assert len(other_frames) == count
    assert len(received) == len(sent) >= 30
    assert [index for index, _, _ in received] == list(range(len(sent)))
    assert all(stamp == sent[index][0] for index, stamp, _ in received)
    assert max(arrival - sent[index][1] for index, _, arrival in received) < 0.4

    # Check the return path of time synchronization under the same input load.
    decoder = SerialFrameDecoder()
    while True:
        try:
            decoder.feed(os.read(master, 4096))
        except BlockingIOError:
            break
    replies = []
    while True:
        packet = decoder.next_packet(time.monotonic())
        if packet is None:
            break
        if packet[0] == TopicInfo.ID_TIME:
            replies.append(struct.unpack('<II', packet[1]))
    assert len(replies) >= 6
    assert all(seconds > 0 and nanoseconds < 1000000000 for seconds, nanoseconds in replies)
    node.destroy_subscription(subscription)


def test_installed_entrypoint_shutdown_releases_uart(tmp_path):
    from serial import Serial
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    log_path = tmp_path / 'serial_node.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'rosbot_description', 'rosserial_node.py', '--ros-args',
            '-p', 'port:=' + slave_name, '-p', 'baud:=115200',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if 'Requesting topics' in log_path.read_text():
                    break
                time.sleep(0.02)
            else:
                pytest.fail('Installed node did not start:\n' + log_path.read_text())
            os.killpg(process.pid, signal.SIGINT)
            assert process.wait(timeout=6.0) == 0, log_path.read_text()
            assert 'Traceback' not in log_path.read_text()
            with Serial(slave_name, 115200, timeout=0, exclusive=True) as port:
                assert port.is_open
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            os.close(master)
            os.close(slave)
