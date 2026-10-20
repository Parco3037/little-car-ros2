#!/usr/bin/env python3
"""Reject a signed/invalid position at the vendor uint16 ROS boundary.

No serial, firmware, pose, offset, torque, or motor changes. Keep the received
value in the warning; never clamp, wrap, or invent a position to fill a reply.
"""
import argparse
from pathlib import Path

OLD = '''            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
'''
NEW = '''            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                # RAICOM_SERVO_POSITION_RESPONSE_V1: uint16 cannot carry negatives.
                if (isinstance(state, (list, tuple)) and len(state) == 1
                        and type(state[0]) is int and 0 <= state[0] <= 65535):
                    data.position = state
                else:
                    self.get_logger().warning(
                        f'Servo {i.id} position reply invalid for uint16: {state!r}')
                    response.state = states + [data]
                    response.success = False
                    return response
'''


def patched(text):
    if NEW in text:
        return text
    if text.count(OLD) != 1:
        raise ValueError('Vendor position response differs; refusing patch')
    result = text.replace(OLD, NEW, 1)
    compile(result, '<servo-position-response>', 'exec')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('node', type=Path)
    args = parser.parse_args()
    path = args.node.resolve(strict=True)
    before = path.read_text(encoding='utf-8')
    after = patched(before)
    if before != after:
        path.write_text(after, encoding='utf-8')
    print('SERVO_POSITION_RESPONSE_READY', path)
