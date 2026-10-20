#!/usr/bin/env python3
"""Narrow, idempotent vendor SDK patch; changes no firmware or motor protocol."""
import argparse
from pathlib import Path

OLD = '''    def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):
        with self.servo_read_lock:
            self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id])
            data = self.bus_servo_queue.get(block=True)
            servo_id, cmd, success, *info = struct.unpack(unpack, data)
            if success == 0:
                return info
'''

NEW = '''    def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):
        # RAICOM_BOUNDED_SERVO_READ_V1: a lost reply must not hang motor callbacks.
        if not self.servo_read_lock.acquire(timeout=0.2):
            return None
        try:
            # Discard already queued replies, never accept one as a new reading.
            for _ in range(128):
                try:
                    self.bus_servo_queue.get_nowait()
                except queue.Empty:
                    break
            else:
                return None
            self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id])
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                try:
                    data = self.bus_servo_queue.get(
                        timeout=max(0.001, deadline - time.monotonic()))
                except queue.Empty:
                    return None
                try:
                    reply_id, reply_cmd, success, *info = struct.unpack(unpack, data)
                except struct.error:
                    continue
                if reply_cmd != cmd or (servo_id != 254 and reply_id != servo_id):
                    continue
                return info if success == 0 else None
            return None
        finally:
            self.servo_read_lock.release()
'''


def patched(text):
    if NEW in text:
        return text
    if text.count(OLD) != 1:
        raise ValueError('SDK read method differs from inspected version; refusing patch')
    result = text.replace(OLD, NEW, 1)
    compile(result, '<patched-sdk>', 'exec')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sdk', type=Path)
    args = parser.parse_args()
    path = args.sdk.resolve(strict=True)
    before = path.read_text(encoding='utf-8')
    after = patched(before)
    if before != after:
        path.write_text(after, encoding='utf-8')
    print('BOUNDED_SERVO_READ_READY', path)
