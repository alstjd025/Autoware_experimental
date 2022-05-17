# Copyright 2020-2022, The Autoware Foundation
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

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    lgsvl_sim_pkg_prefix = get_package_share_directory('lgsvl_simulation')

    lgsvl_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lgsvl_interface.param.yaml')
    lgsvl_sim_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lgsvl_simulation.param.yaml')
    map_publisher_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/map_publisher_sim.param.yaml')
    ndt_localizer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/ndt_localizer_sim.param.yaml')
    pc_filter_transform_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/pc_filter_transform.param.yaml')
    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')
    lat_control_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lateral_controller.param.yaml')
    lon_control_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/longitudinal_controller.param.yaml')
    latlon_muxer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/latlon_muxer.param.yaml')
    pure_pursuit_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/pure_pursuit.param.yaml')
    controller_testing_param_file = os.path.join(
        avp_demo_pkg_prefix, "param/defaults.param.yaml"
    )
    mpc_controller_param_file = os.path.join(
        avp_demo_pkg_prefix, "param/mpc_defaults.yaml"
    )

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    map_pcd_file = os.path.join("","/opt/AutowareAuto/share/autoware_demos/data/autonomoustuff_parking_lot.pcd")

    # map_pcd_file = os.path.join(
    #     avp_demo_pkg_prefix, 'data/autonomoustuff_parking_lot_lgsvl.pcd')
    map_yaml_file = os.path.join(
        avp_demo_pkg_prefix, 'data/autonomoustuff_parking_lot_lgsvl.yaml')
    print(map_pcd_file)
    # Arguments

    with_lgsvl_param = DeclareLaunchArgument(
        'with_lgsvl',
        default_value='true',
        description='Launch simulation on the LGSVL simulator through the API',
    )
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )
    lat_control_param = DeclareLaunchArgument(
        'lat_control_param_file',
        default_value=lat_control_param_file,
        description='Path to config file for lateral controller'
    )
    lon_control_param = DeclareLaunchArgument(
        'lon_control_param_file',
        default_value=lon_control_param_file,
        description='Path to config file for longitudinal controller'
    )
    latlon_muxer_param = DeclareLaunchArgument(
        'latlon_muxer_param_file',
        default_value=latlon_muxer_param_file,
        description='Path to config file for lateral and longitudinal control commands muxer'
    )
    controller_testing_param = DeclareLaunchArgument(
        "controller_testing_param_file",
        default_value=controller_testing_param_file,
        description="Path to config file for Controller Testing",
    )
    mpc_controller_param = DeclareLaunchArgument(
        "mpc_controller_param_file",
        default_value=mpc_controller_param_file,
        description="Path to config file to MPC Controller",
    )
    pure_pursuit_controller_param = DeclareLaunchArgument(
        'pure_pursuit_param_file',
        default_value=pure_pursuit_param_file,
        description='Path to config file for pp controller'
    )
    real_time_sim_param = DeclareLaunchArgument(
        'real_time_sim',
        default_value='False',
        description='Launch RVIZ2 in addition to other nodes'
    )
    show_final_report_param = DeclareLaunchArgument(
        'show_final_report',
        default_value='False',
        description='Show graphical report of final results'
    )
    controller_testing_params = [
        LaunchConfiguration("controller_testing_param_file"),
        {
            'real_time_sim': LaunchConfiguration('real_time_sim'),
            'show_final_report': LaunchConfiguration('show_final_report')
        }
    ]

    

    # Nodes

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        name='lgsvl_interface_node',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file'),
          {"lgsvl.publish_tf": True},
          {"lgsvl.control_command" : "ackerman"}
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
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_xyzi")]
    )
    filter_transform_vlp16_rear = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_rear',
        namespace='lidar_rear',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_xyzi")]
    )
    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file'),
                    {"map_pcd_file": map_pcd_file,
                     "map_yaml_file": map_yaml_file}]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[
            LaunchConfiguration('ndt_localizer_param_file'),
            # Use preset initial pose, if using a preset simulation
            {'load_initial_pose_from_parameters': LaunchConfiguration('with_lgsvl')},
        ],
        remappings=[
            ("points_in", "/lidars/points_fused_downsampled"),
            ("observation_republish", "/lidars/points_fused_viz"),
        ]
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
    latlon_muxer = Node(
        package='trajectory_follower_nodes',
        executable='latlon_muxer_node_exe',
        name='latlon_muxer_node',
        namespace='control',
        parameters=[
            LaunchConfiguration('latlon_muxer_param_file'),
        ],
        remappings=[
           ("input/lateral/control_cmd", "output/lateral/control_cmd"),
           ("input/longitudinal/control_cmd", "output/longitudinal/control_cmd"),
           ("output/control_cmd", "/vehicle/ackermann_vehicle_command"),
        ],
    )
    controller_testing = Node(
        package="controller_testing",
        executable="controller_testing_main.py",
        namespace="control",
        name="controller_testing_node",
        parameters=controller_testing_params,
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/vehicle/control_command"),
            ("control_diagnostic", "/control/control_diagnostic"),
        ],
    )
    # pure_pursuit_controller = Node(
    #     package='pure_pursuit_nodes',
    #     executable='pure_pursuit_node_exe',
    #     name='pure_pursuit_node',
    #     namespace='control',
    #     parameters=[
    #         LaunchConfiguration('pure_pursuit_param_file'), {},
    #     ],
    #     remappings=[
    #         ("current_pose", "/vehicle/vehicle_kinematic_state"),
    #         ("trajectory", "/planning/trajectory"),
    #         ("ctrl_cmd", "/vehicle/control_command"),
    #         ("ctrl_diag", "/control/control_diagnostic"),
    #     ],
    # )

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

        # mpc_controller
    mpc_controller = Node(
        package="mpc_controller_nodes",
        executable="mpc_controller_node_exe",
        # namespace="control",
        name="mpc_controller_node",
        output="screen",
        parameters=[LaunchConfiguration("mpc_controller_param_file")],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/control_command"),
            ("control_diagnostic", "/control/control_diagnostic"),
        ],
    )



    lgsvl_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lgsvl_sim_pkg_prefix, '/launch/sim.launch.py']),
        launch_arguments={
            'simulation_params': LaunchConfiguration(
                'simulation_params',
                default=lgsvl_sim_param_file
            )
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_lgsvl')),
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([avp_demo_pkg_prefix, '/launch/avp_core.launch.py']),
        launch_arguments={}.items()
    )

    point_type_adapter_pkg_prefix = get_package_share_directory(
        'point_type_adapter')

    adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_type_adapter_pkg_prefix,
                         'launch/point_type_adapter.launch.py'))
    )

    return LaunchDescription([
        with_lgsvl_param,
        lgsvl_interface_param,
        map_publisher_param,
        ndt_localizer_param,
        pc_filter_transform_param,
        vehicle_characteristics_param,
        
        #for LL
        #lat_control_param,
        #lon_control_param,
        #latlon_muxer_param,
        
        #Minsung : If you want to change controller
        #          check lgsvl.param.yaml

        #For MPC
        mpc_controller_param,
        mpc_controller,
        

        #For PP
        #pure_pursuit_controller_param,
        #pure_pursuit_controller,
        
        #for LL
        #lat_control,
        #lon_control,
        #latlon_muxer,
        
        urdf_publisher,
        lgsvl_interface,
        map_publisher,
        ndt_localizer,
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        lgsvl_simulation_launch,
        core_launch,
        adapter_launch,
    ])
