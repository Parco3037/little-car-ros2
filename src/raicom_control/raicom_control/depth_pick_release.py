"""Native grasp with platform clearance and sag allowance; exact runtime binding."""
import hashlib
from importlib.util import find_spec
from pathlib import Path

RELEASE = 'rgbd-platform-comp-20260923-u320-floor'
COMMAND = ['ros2', 'launch', 'rgbd_grasp', 'rgbd_grasp.launch.py',
           'start_servo_controller:=false', 'start_hardware:=false',
           'start_description:=false']
HASHES = {'grasp_once_node.py': 'd6f4e7c09735c105e53fdbcabb610222b1c183666a1c4ccf8e954364fccd9a23', 'grasp_hardware.py': 'f0b117a9359e0d9b0347f4dc6241e39334a93f7cb2ab30a6009c12b35ffe0752', 'target_to_ik_node.py': 'de567015f205fd4fb3be0f5108920c5ccae6b5c3a4515e94ffc9b442c4cdaed9', 'grasp_core.py': '4e346d0ec2413101e63ad731fdc7b40793f01e4122d142107abd203578b0a586', 'blue_block_3d_node.py': '2fff758b3f4a77fbac699257e05060e9a7541af1073ad6664a96f798cdbf0e44', 'grasp_clearance.py': 'bc59a83f346c0aa04e6fa8d709a74ff50ea0d968e5c1bdae8c5ba834a07f521b'}


def verify_release():
    spec = find_spec('rgbd_grasp')
    if spec is None or not spec.origin:
        raise RuntimeError('未安装已固定的 RGB-D 抓取程序')
    root = Path(spec.origin).resolve().parent
    result = {}
    for name, expected in HASHES.items():
        digest = hashlib.sha256((root/name).read_bytes()).hexdigest()
        if digest != expected:
            raise RuntimeError('抓取程序版本与固定版本不一致：' + name)
        result[name] = digest
    from ament_index_python.packages import get_package_share_directory
    launch = Path(get_package_share_directory('rgbd_grasp'))/'launch/rgbd_grasp.launch.py'
    if hashlib.sha256(launch.read_bytes()).hexdigest() != 'af9f8345a944e6fb3452f291c32866b1ac416939967fb3681437fa92f1d8d16a':
        raise RuntimeError('抓取启动入口与固定版本不一致')
    return dict(release=RELEASE, source=str(root), files=result)
