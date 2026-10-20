#!/usr/bin/env python3
"""One camera-only ROS/HTTP preview on private DDS 88; no motion interfaces."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from io import BytesIO

assert os.environ.get('ROS_DOMAIN_ID')=='88' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from PIL import Image
from raicom_control.vision_ros_client import VisionClient


def main():
    out=Path(sys.argv[1]);out.mkdir(exist_ok=True,parents=True)
    rclpy.init();node=rclpy.create_node('preview_image_pair_probe')
    client=VisionClient(node,with_images=True);children=[];logs=[]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03);client.poll()
    try:
        spin(.8)
        assert set(node.get_node_names())=={'preview_image_pair_probe'},'Private test domain occupied'
        for kind in ['camera','infer']:
            log=(out/(kind+'.log')).open('w');logs.append(log)
            children.append(subprocess.Popen([sys.executable,'-c',
                'from raicom_control.vision_ros_nodes import run;run('+repr(kind)+')'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+10
        while not client.ready() and time.monotonic()<deadline:spin(.1)
        assert client.ready(),'Adapter discovery failed'
        reply=None
        for attempt in range(3):
            spin(.4)
            future=client.request('image-pair-test')
            deadline=time.monotonic()+12
            while not future.done() and time.monotonic()<deadline:spin(.05)
            try:reply=future.result();break
            except RuntimeError as exc:
                if 'busy' not in str(exc):raise
        assert reply is not None
        packet=None;deadline=time.monotonic()+3
        while packet is None and time.monotonic()<deadline:
            spin(.05);packet=client.take_preview()
        assert packet is not None,reply
        image=Image.open(BytesIO(packet['jpeg']))
        assert image.format=='JPEG' and list(image.size)==reply['payload']['image_size']
        (out/'live_input.jpg').write_bytes(packet['jpeg'])
        report={k:v for k,v in packet.items() if k!='jpeg'}
        report.update(jpeg_bytes=len(packet['jpeg']),test='PASS',classification_accepted=False,motion_commands=False)
        (out/'live_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False))
    finally:
        client.close()
        for p in children:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
        for stream in logs:stream.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
