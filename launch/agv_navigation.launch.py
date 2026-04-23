"""
agv_navigation.launch.py
────────────────────────
ONE launch file to rule them all:
  Hardware + Localization + NAV2 + RViz (pre-configured)

Usage:
  ros2 launch agv_base_controller agv_navigation.launch.py
  ros2 launch agv_base_controller agv_navigation.launch.py map:=/path/to/map.yaml esp32_ip:=192.168.x.x
"""

import os
import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('agv_base_controller')

    # ───────── Arguments ─────────
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip', default_value='192.168.4.1') # ESP32 ip
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.expanduser('~/ros2_ws/maps/my_map.yaml'))

    esp32_ip = LaunchConfiguration('esp32_ip')
    map_file = LaunchConfiguration('map')

    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'nav2_view.rviz')

    # ───────── Hardware ─────────
    udp_bridge = Node(
        package='agv_base_controller',
        executable='udp_bridge_node',
        name='udp_bridge_node',
        output='screen',
        parameters=[{
            'esp32_ip': esp32_ip, 'esp32_port': 8889, 'pc_port': 8888,
            'wheel_base': 0.285, 'wheel_radius': 0.0319,
            'ticks_per_rev': 330.0, 'publish_tf': True,
        }]
    )

    lidar_node = Node(
        package='sllidar_ros2', executable='sllidar_node',
        name='sllidar_node', output='screen',
        parameters=[{
            'channel_type': 'tcp', 'tcp_ip': esp32_ip, 'tcp_port': 20108,
            'serial_baudrate': 115200, 'frame_id': 'laser',
            'angle_compensate': True, 'scan_mode': '',
            'inverted': False, 'scan_frequency': 10.0,
        }]
    )

    base_to_laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0.085', '--y', '-0.01', '--z', '0.175',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser']
    )

    # ───────── Map Server (lifecycle) ─────────
    map_server_node = LifecycleNode(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen', namespace='',
        parameters=[nav2_config, {'yaml_filename': map_file}],
    )
    emit_cfg_map = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(map_server_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    act_map = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=map_server_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    # ───────── AMCL (lifecycle) ─────────
    amcl_node = LifecycleNode(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen', namespace='',
        parameters=[nav2_config],
    )
    emit_cfg_amcl = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(amcl_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    act_amcl = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=amcl_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    # ───────── NAV2 Core ─────────
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_config]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_config]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_config]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_config]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_config]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[nav2_config]),
    ]

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': False, 'autostart': True,
            'node_names': [
                'controller_server', 'planner_server', 'behavior_server',
                'bt_navigator', 'waypoint_follower', 'velocity_smoother',
            ],
        }],
    )

    # ───────── RViz (pre-configured) ─────────
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        esp32_ip_arg, map_arg,
        # Hardware
        udp_bridge, lidar_node, base_to_laser_tf,
        # Map server
        act_map, map_server_node, emit_cfg_map,
        # AMCL
        act_amcl, amcl_node, emit_cfg_amcl,
        # NAV2
        *nav2_nodes, lifecycle_mgr,
        # RViz
        rviz_node,
    ])