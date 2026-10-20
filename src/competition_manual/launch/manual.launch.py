from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import glob
import os


def array_port():
    # The vendor serial protocol belongs to the XF array, not every CH340 USB mic.
    try:
        with open('/proc/asound/cards', encoding='ascii') as cards:
            if 'XFMDPV0018' not in cards.read():
                return None
    except OSError:
        return None
    if os.path.exists('/dev/ring_mic'):
        return '/dev/ring_mic'
    matches = []
    for port in glob.glob('/dev/ttyUSB*'):
        device = os.path.realpath('/sys/class/tty/' + os.path.basename(port))
        for parent in (device, *[os.path.join(device, *(['..'] * n)) for n in range(1, 6)]):
            try:
                with open(os.path.join(parent, 'idVendor'), encoding='ascii') as vendor:
                    vendor_id = vendor.read().strip().lower()
                with open(os.path.join(parent, 'idProduct'), encoding='ascii') as product:
                    product_id = product.read().strip().lower()
            except OSError:
                continue
            if (vendor_id, product_id) == ('1a86', '7523'):
                matches.append(port)
                break
    return matches[0] if len(matches) == 1 else None


def launch_array_wake(context):
    port = LaunchConfiguration('array_port').perform(context)
    if port == 'auto':
        port = array_port()
    if not port or not os.path.exists(port):
        print('array wake unavailable: connect the microphone or set array_port')
        return []
    return [Node(
        package='xf_mic_asr_offline', executable='awake_node.py',
        name='awake_node', output='screen',
        parameters=[{
            'port': port,
            'mic_type': 'mic6_circle',
            'awake_word': 'xu2 zong3',
            'enable_setting': True,
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('competition_manual')
    return LaunchDescription([
        DeclareLaunchArgument('array_port', default_value='auto'),
        OpaqueFunction(function=launch_array_wake),
        Node(
            package='competition_manual', executable='joy_reconnect',
            name='competition_joy_reconnect', output='screen',
            respawn=True, respawn_delay=1.0,
            parameters=[{
                'input_device_name': 'Xbox Wireless Controller',
                'sdl_device_name': 'Xbox One Elite 2 Controller',
            }],
        ),
        Node(
            package='competition_manual', executable='xbox_teleop',
            name='competition_xbox_teleop', output='screen',
            parameters=[{
                'max_linear_x': 0.5,
                'max_linear_y': 0.5,
                'max_angular_z': 1.5,
                'deadzone': 0.05,
                'start_sound': os.path.join(share, 'sounds', 'teleop_done.wav'),
            }],
        ),
        Node(
            package='competition_manual', executable='voice_node',
            name='competition_voice', output='screen',
            parameters=[{
                'model_path': os.path.join(share, 'model'),
                'wake_sound': os.path.join(share, 'sounds', 'ganma.wav'),
                'auto_sound': os.path.join(
                    get_package_share_directory('xf_mic_asr_offline'),
                    'feedback_voice', 'ok.wav'),
                'use_array_wake': True,
            }],
        ),
        Node(
            package='competition_manual', executable='auto_advance',
            name='competition_auto_advance', output='screen',
            parameters=[{'speed_mps': 0.2, 'duration_sec': 2.7}],
        ),
    ])
