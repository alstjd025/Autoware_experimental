# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch modules for behavior planner."""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from pathlib import Path


def generate_launch_description():
    param_path = Path(get_package_share_directory('autoware_demos')) / 'param'

    urdf_pkg_prefix = Path(get_package_share_directory('lexus_rx_450h_description'))
    urdf_path = urdf_pkg_prefix / 'urdf' / 'lexus_rx_450h.urdf'
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    map_pcd_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'data/autonomoustuff_parking_lot_lgsvl.pcd')
    map_yaml_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'data/autonomoustuff_parking_lot_lgsvl.yaml')
    map_osm_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'data/autonomoustuff_parking_lot.osm')

    vehicle_characteristics_param_file = os.path.join(
        get_package_share_directory('autoware_demos'), 'param/vehicle_characteristics.param.yaml')

    vehicle_constants_manager_param_file = os.path.join(
        get_package_share_directory('autoware_auto_launch'), 'param/lexus_rx_hybrid_2016.param.yaml')

    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='False',
        description='Enable obstacle detection'
    )

    object_collision_estimator_param_file = os.path.join(
        get_package_share_directory('autoware_auto_launch'), 'param/object_collision_estimator.param.yaml')

    pure_pursuit_param_file = os.path.join(
        get_package_share_directory("autoware_demos"), 'param/pure_pursuit.param.yaml')

    lat_control_param_file = os.path.join(
        get_package_share_directory("autoware_demos"), 'param/avp/lateral_controller.param.yaml')
    lon_control_param_file = os.path.join(
        get_package_share_directory("autoware_demos"), 'param/avp/longitudinal_controller.param.yaml')
    latlon_muxer_param_file = os.path.join(
        get_package_share_directory("autoware_demos"), 'param/avp/latlon_muxer.param.yaml')

    controller_testing_param_file = os.path.join(
        get_package_share_directory('autoware_demos'), "param/defaults.param.yaml"
    )

    # Nodes

    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )
    vehicle_constants_manager_param = DeclareLaunchArgument(
        'vehicle_constants_manager_param_file',
        default_value=vehicle_constants_manager_param_file,
        description='Path to parameter file for vehicle_constants_manager'
    )

    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=str(param_path / 'behavior_planner.param.yaml'),
        description='Path to paramter file for behavior planner'
    )

    behavior_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('behavior_planner_param_file'),
            {'enable_object_collision_estimator': LaunchConfiguration('with_obstacles')},
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('gear_report', '/vehicle/gear_report'),
            ('gear_command', '/vehicle/gear_command')
        ]
    )

    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=object_collision_estimator_param_file,
        description='Path to parameter file for object collision estimator'
    )

    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        name='object_collision_estimator_node',
        namespace='planning',
        executable='object_collision_estimator_node_exe',
        parameters=[
            LaunchConfiguration('object_collision_estimator_param_file'),
            {
                'target_frame_id': "map"
            },
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
            ('predicted_objects', '/prediction/predicted_objects'),

        ],
        condition=IfCondition(LaunchConfiguration('with_obstacles'))
    )

    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )

    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=str(param_path / 'lane_planner.param.yaml'),
        description='Path to parameter file for lane planner'
    )

    lane_planner = Node(
        package='lane_planner_nodes',
        name='lane_planner_node',
        namespace='planning',
        executable='lane_planner_node_exe',
        parameters=[
            LaunchConfiguration('lane_planner_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=str(param_path / 'lanelet2_map_provider.param.yaml'),
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file'),
                    {'map_osm_file': map_osm_file}]
    )

    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        namespace='had_maps'
    )

    map_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["-57.60810852050781", "-41.382755279541016", "0", "0", "0", "0", "map", "odom"]
    )

    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=str(param_path / 'map_publisher.param.yaml'),
        description='Path to config file for Map Publisher'
    )

    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file'),
                    {"map_pcd_file": map_pcd_file,
                     "map_yaml_file": map_yaml_file}]
    )

    lat_control_param = DeclareLaunchArgument(
        'lat_control_param_file',
        default_value=lat_control_param_file,
        description='Path to config file for lateral controller'
    )

    lat_control = Node(
        package='trajectory_follower_nodes',
        executable='lateral_controller_node_exe',
        name='lateral_controller_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('lat_control_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
           ("input/reference_trajectory", "/planning/trajectory"),
           ("input/current_kinematic_state", "/vehicle/vehicle_kinematic_state"),
           ("input/tf", "/tf"),
           ("input/tf_static", "/tf_static"),
        ],
    )

    lon_control_param = DeclareLaunchArgument(
        'lon_control_param_file',
        default_value=lon_control_param_file,
        description='Path to config file for longitudinal controller'
    )

    lon_control = Node(
        package='trajectory_follower_nodes',
        executable='longitudinal_controller_node_exe',
        name='longitudinal_controller_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('lon_control_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
           ("input/current_trajectory", "/planning/trajectory"),
           ("input/current_state", "/vehicle/vehicle_kinematic_state"),
           ("input/tf", "/tf"),
           ("input/tf_static", "/tf_static"),
        ],
    )

    latlon_muxer_param = DeclareLaunchArgument(
        'latlon_muxer_param_file',
        default_value=latlon_muxer_param_file,
        description='Path to config file for lateral and longitudinal control commands muxer'
    )

    latlon_muxer = Node(
        package='trajectory_follower_nodes',
        executable='latlon_muxer_node_exe',
        name='latlon_muxer_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('latlon_muxer_param_file'),
            {"control_command": "ackermann"}  # publishes odom-base_link
        ],
        remappings=[
           ("input/lateral/control_cmd", "output/lateral/control_cmd"),
           ("input/longitudinal/control_cmd", "output/longitudinal/control_cmd"),
           ("output/control_cmd", "/vehicle/ackermann_vehicle_command"),
        ],
    )

    mpc_controller_param = DeclareLaunchArgument(
        'mpc_controller_param_file',
        default_value=str(param_path / 'mpc_controller.param.yaml'),
        description='Path to config file for MPC'
    )
    mpc_controller = Node(
        package='mpc_controller_nodes',
        executable='mpc_controller_node_exe',
        name='mpc_controller',
        namespace='control',
        parameters=[
            LaunchConfiguration('mpc_controller_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ]
    )

    pure_pursuit_controller_param = DeclareLaunchArgument(
        'pure_pursuit_param_file',
        default_value=pure_pursuit_param_file,
        description='Path to config file for pp controller'
    )

    pure_pursuit_controller = Node(
        package='pure_pursuit_nodes',
        executable='pure_pursuit_node_exe',
        name='pure_pursuit_node',
        output="screen",
        parameters=[
            LaunchConfiguration('pure_pursuit_param_file'),
        ],
        remappings=[
            ("current_pose", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/vehicle_command"),
            ("ctrl_diag", "/control/control_diagnostic"),
        ],
    )

    controller_testing_param = DeclareLaunchArgument(
        "controller_testing_param_file",
        default_value=controller_testing_param_file,
        description="Path to config file for Controller Testing",
    )

    controller_testing = Node(
        package="controller_testing",
        executable="controller_testing_main.py",
        namespace="control",
        name="controller_testing_node",
        parameters=[LaunchConfiguration('controller_testing_param_file')],
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/vehicle/control_command"),
            ("control_diagnostic", "/control/control_diagnostic"),
        ],
    )

    costmap_generator_param = DeclareLaunchArgument(
        'costmap_generator_param_file',
        default_value=os.path.join(param_path, 'costmap_generator.param.yaml'),
        description='Path to parameter file for costmap generator'
    )
    costmap_generator = Node(
        package='costmap_generator_nodes',
        executable='costmap_generator_node_exe',
        name='costmap_generator_node',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('costmap_generator_param_file'),
        ],
        remappings=[
            ('~/client/HAD_Map_Service', '/had_maps/HAD_Map_Service')
        ]
    )

    freespace_planner_param = DeclareLaunchArgument(
        'freespace_planner_param_file',
        default_value=os.path.join(param_path, 'freespace_planner.param.yaml'),
        description='Path to parameter file for freespace_planner'
    )

    freespace_planner = Node(
        package='freespace_planner_nodes',
        executable='freespace_planner_node_exe',
        name='freespace_planner',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('freespace_planner_param_file'),
            LaunchConfiguration('vehicle_constants_manager_param_file')
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(param_path.parent / 'rviz2' / 'test.rviz')],
        remappings=[("initialpose", "/localization/initialpose"),
                    ("move_base_simple/goal", "/planning/goal_pose")],
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=str(param_path / 'lgsvl_interface.param.yaml'),
        description='Path to config file for LGSVL Interface'
    )

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file'),
          {"lgsvl.publish_tf": True},  # publishes odom-base_link
        #  {"control_command" : "ackermann"}, #for LL
          {"control_command" : "basic"}, #for pp, mpc
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    return LaunchDescription([
        vehicle_characteristics_param,
        vehicle_constants_manager_param,
        with_obstacles_param,
        controller_testing_param,
        behavior_planner_param,
        behavior_planner,
        global_planner,
        lane_planner_param,
        lane_planner,
        lanelet2_map_provider_param,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        lgsvl_interface_param,
        lgsvl_interface,
        map_odom_publisher,
        map_publisher_param,
        map_publisher,

        #for LL
        #lat_control_param,
        #lon_control_param,
        #latlon_muxer_param,
        #lat_control,
        #lon_control,
        #latlon_muxer,

        #For MPC
        mpc_controller_param,
        mpc_controller,
        
        #For PP
        #pure_pursuit_controller_param,
        #pure_pursuit_controller,

        costmap_generator_param,
        costmap_generator,
        freespace_planner_param,
        freespace_planner,
        object_collision_estimator_param,
        object_collision_estimator,
        rviz2,
        # odom_bl_publisher,
        urdf_publisher,
    ])
