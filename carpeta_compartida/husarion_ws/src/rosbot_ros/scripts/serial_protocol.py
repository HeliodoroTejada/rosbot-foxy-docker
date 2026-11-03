"""Incremental rosserial v2 framing; incomplete UART reads never wait for bytes."""
from collections import Counter


class SerialFrameDecoder:
    SYNC = b'\xff\xfe'
    MAX_BUFFER = 2 * (65535 + 8)

    def __init__(self, timeout=5.0):
        self.timeout = timeout
        self.buffer = bytearray()
        self.started = None
        self.errors = Counter()

    def reset(self):
        self.buffer.clear()
        self.started = None

    def feed(self, data):
        if len(self.buffer) + len(data) > self.MAX_BUFFER:
            self.reset()
            self.errors['overflow'] += 1
        self.buffer.extend(data[-self.MAX_BUFFER:])

    def _discard(self, count, reason):
        del self.buffer[:count]
        self.started = None
        self.errors[reason] += 1

    def next_packet(self, now):
        """Return one (topic_id, payload), or None until a complete frame arrives."""
        while self.buffer:
            start = self.buffer.find(self.SYNC)
            if start < 0:
                # A final 0xff can be the first byte of the next frame.
                keep = 1 if self.buffer[-1] == 0xff else 0
                discard = len(self.buffer) - keep
                if discard:
                    self._discard(discard, 'sync')
                return None
            if start:
                self._discard(start, 'sync')
            if self.started is None:
                self.started = now
            if len(self.buffer) < 5:
                if now - self.started >= self.timeout:
                    self._discard(1, 'timeout')
                    continue
                return None
            if sum(self.buffer[2:5]) % 256 != 255:
                self._discard(1, 'length_checksum')
                continue
            length = self.buffer[2] | (self.buffer[3] << 8)
            frame_size = length + 8
            if len(self.buffer) < frame_size:
                if now - self.started >= self.timeout:
                    self._discard(1, 'timeout')
                    continue
                return None
            if sum(self.buffer[5:frame_size]) % 256 != 255:
                self._discard(1, 'data_checksum')
                continue
            topic_id = self.buffer[5] | (self.buffer[6] << 8)
            payload = bytes(self.buffer[7:frame_size - 1])
            del self.buffer[:frame_size]
            self.started = None
            return topic_id, payload
        return None
