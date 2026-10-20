"""Forward fresh stdin heartbeats; EOF, silence and STOP latch an emergency stop."""
import os
import select
import sys
import time
import rclpy
from std_msgs.msg import Empty


def main():
    rclpy.init();node=rclpy.create_node('fixed_route_operator_presence')
    heartbeat=node.create_publisher(Empty,'/raicom/task/heartbeat',1)
    stop=node.create_publisher(Empty,'/raicom/nav_stop',10)
    last=None;buffer=b''
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            if select.select([sys.stdin.buffer],[],[],.02)[0]:
                chunk=os.read(sys.stdin.fileno(),4096)
                if not chunk:break
                buffer+=chunk
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    if line.strip()!=b'alive':return
                    last=time.monotonic();heartbeat.publish(Empty())
                if len(buffer)>4096:break
            if last is not None and time.monotonic()-last>.35:break
    finally:
        for _ in range(3):stop.publish(Empty());rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
