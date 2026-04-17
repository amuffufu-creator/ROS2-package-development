"""
nav2_bringup.launch.py
──────────────────────
Launches full navigation stack on a saved map:
  1. udp_bridge_node      — encoder odom + cmd_vel bridge
  2. sllidar_ros2         — LiDAR driver via TCP to ESP32
  3. static TF            — base_link → laser
  4. map_server           — serves the saved map
  5. AMCL                 — localization on the map
  6. NAV2 stack           — planner, controller, behaviors, etc.

Usage:
  ros2 launch agv_base_controller nav2_bringup.launch.py map:=$HOME/ros2_ws/maps/my_map.yaml
  ros2 launch agv_base_controller nav2_bringup.launch.py map:=$HOME/ros2_ws/maps/my_map.yaml esp32_ip:=192.168.1.100
"""

import os
import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('agv_base_controller')

    # ───────── Launch arguments ─────────
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip', default_value='192.168.100.25',
        description='ESP32 IP address'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/ros2_ws/maps/my_map.yaml'),
        description='Path to map yaml file'
    )

    esp32_ip = LaunchConfiguration('esp32_ip')
    map_file = LaunchConfiguration('map')

    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # ───────── 1. UDP Bridge Node ─────────
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

    # ───────── 2. RPLIDAR A1M8 ─────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'tcp',
            'tcp_ip':           esp32_ip,
            'tcp_port':         20108,
            'serial_baudrate':  115200,
            'frame_id':         'laser',
            'angle_compensate': True,
            'scan_mode':        '',
            'inverted':         False,
            'scan_frequency':   10.0,
        }]
    )

    # ───────── 3. Static TF: base_link → laser ─────────
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.21', '--y', '0.0', '--z', '0.2162',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ]
    )

    # ───────── 4. Map Server (lifecycle) ─────────
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_config, {'yaml_filename': map_file}],
        namespace='',
    )

    emit_configure_map = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # ───────── 5. AMCL (lifecycle) ─────────
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config],
        namespace='',
    )

    emit_configure_amcl = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_amcl_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=amcl_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(amcl_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # ───────── 6. NAV2 Core Nodes ─────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config],
    )

    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        esp32_ip_arg,
        map_arg,
        # Hardware
        udp_bridge,
        lidar_node,
        base_to_laser_tf,
        # Map server lifecycle
        activate_map_on_configured,
        map_server_node,
        emit_configure_map,
        # AMCL lifecycle
        activate_amcl_on_configured,
        amcl_node,
        emit_configure_amcl,
        # NAV2 core (managed by lifecycle_manager)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_nav,
    ])