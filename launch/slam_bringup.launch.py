"""
slam_bringup.launch.py
──────────────────────
Launches everything needed for SLAM:
  1. udp_bridge_node      — encoder odom + cmd_vel bridge
  2. sllidar_ros2         — LiDAR driver via TCP to ESP32
  3. static TF            — base_link → laser
  4. slam_toolbox         — online async SLAM (lifecycle, auto-activated)

Usage:
  ros2 launch agv_base_controller slam_bringup.launch.py
  ros2 launch agv_base_controller slam_bringup.launch.py esp32_ip:=192.168.100.25
"""

import os
import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ───────── Launch arguments ─────────
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip', default_value='192.168.100.25',
        description='ESP32 IP address on WiFi'
    )
    esp32_ip = LaunchConfiguration('esp32_ip')

    # ───────── 1. UDP Bridge Node (odom + cmd_vel) ─────────
    udp_bridge = Node(
        package='agv_base_controller',
        executable='udp_bridge_node',
        name='udp_bridge_node',
        output='screen',
        parameters=[{
            'esp32_ip':      esp32_ip,
            'esp32_port':    8889,
            'pc_port':       8888,
            'wheel_base':    0.285,
            'wheel_radius':  0.0319,
            'ticks_per_rev': 330.0,
            'publish_tf':    True,
        }]
    )

    # ───────── 2. RPLIDAR A1M8 Driver (TCP bridge to ESP32) ─────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':       'tcp',
            'tcp_ip':             esp32_ip,
            'tcp_port':           20108,
            'serial_baudrate':    115200,
            'frame_id':           'laser',
            'angle_compensate':   True,
            'scan_mode':          '',
            'inverted':           False,
            'scan_frequency':     10.0,
        }]
    )

    # ───────── 3. Static TF: base_link → laser ─────────
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x',    '0.21',
            '--y',    '0.0',
            '--z',    '0.2162',
            '--roll',  '0',
            '--pitch', '0',
            '--yaw',   '0',
            '--frame-id',       'base_link',
            '--child-frame-id', 'laser',
        ]
    )

    # ───────── 4. SLAM Toolbox (lifecycle node) ─────────
    slam_config = os.path.join(
        get_package_share_directory('agv_base_controller'),
        'config',
        'slam_toolbox_params.yaml'
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
        namespace='',
    )

    # Auto-configure slam_toolbox when it starts
    emit_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate slam_toolbox after it finishes configuring
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        esp32_ip_arg,
        udp_bridge,
        lidar_node,
        base_to_laser_tf,
        # Slam lifecycle: register handler first, then node, then configure event
        activate_on_configured,
        slam_node,
        emit_configure,
    ])