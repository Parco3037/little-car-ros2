"""Teammate ROS2 String/JSON example; manual-load adapter needs explicit flags.

Start: python3 -m raicom_control.pick_example
The teammate replaces perform_pick(), returning only measured outcomes. For
manual block loading run with --manual; confirmations are typed at the console.
"""
import argparse
import json
import queue
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .delivery_protocol import identity


class PickExample(Node):
    def __init__(self,manual=False):
        super().__init__('raicom_pick_example')
        self.manual=manual;self.current=None;self.seen=set();self.replies=queue.Queue()
        self.result=self.create_publisher(String,'/raicom/pick/result',10)
        self.create_subscription(String,'/raicom/pick/request',self.request,10)
        self.create_timer(.05,self.flush)
        if manual:threading.Thread(target=self.console,daemon=True).start()

    def request(self,msg):
        try:
            req=json.loads(msg.data);key=identity(req)
            if req.get('command')=='cancel':
                self.seen.add(key)
                if self.current and identity(self.current)==key:self.current=None
                return
            if req.get('command')!='start' or key in self.seen:return
            if self.current is not None:return
            self.current=req;self.seen.add(key)
            self.get_logger().info('PICK_REQUEST '+json.dumps(req))
            if self.manual:
                self.get_logger().info('装入真实方块并完成夹紧/交还后，输入 request_id T或G CONFIRM_LOADED_CLAMPED_ARM_RETURNED')
            else:self.perform_pick(req)
        except (ValueError,TypeError):self.get_logger().error('Invalid pick request JSON')

    def perform_pick(self,request):
        # Insert teammate's recognition/alignment/grasp here. Do not return
        # success until hardware grip and control handoff have been verified.
        self.report(request,success=False,reason='teammate implementation required')

    def report(self,request,*,success,category=None,gripper_closed=False,
               holding=False,arm_control_returned=False,reason=''):
        if not self.current or identity(request)!=identity(self.current):return
        payload={k:request[k] for k in ('task_id','request_id','round')}
        payload.update(success=success,category=category,gripper_closed=gripper_closed,
                       holding=holding,arm_control_returned=arm_control_returned,reason=reason)
        self.result.publish(String(data=json.dumps(payload)));self.current=None

    def console(self):
        while rclpy.ok():
            try:self.replies.put(input().strip().split())
            except EOFError:return

    def flush(self):
        while not self.replies.empty():
            words=self.replies.get_nowait()
            if (len(words)==3 and self.current and words[0]==self.current['request_id']
                    and words[1] in ('T','G') and words[2]=='CONFIRM_LOADED_CLAMPED_ARM_RETURNED'):
                self.report(self.current,success=True,category=words[1],gripper_closed=True,
                            holding=True,arm_control_returned=True,reason='operator-confirmed physical load')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--manual',action='store_true')
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=PickExample(args.manual)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
