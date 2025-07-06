from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('sensor_fusion_pkg'),
        'config'
    )

    ekf_odom_config = os.path.join(config_path,'ekf_odom.yaml')
    ekf_map_config = os.path.join(config_path,'ekf_map.yaml')
    navsat_config = os.path.join(config_path,'navsat.yaml')

    return LaunchDescription([
    Node(
        package='sensor_fusion_pkg',
        executable='mtnode',
        name='mtnode',
        output='screen',
        parameters=[
            {'device': '/dev/ttyUSB0'},
            {'baudrate': 115200},
            {'timeout': 0.002},
            {'initial_wait': 0.1},
            
        ]
    ),
    # Node(
    #     package='sensor_fusion_pkg',
    #     executable='odom_publisher_node',
    #     name='odom_publisher_node',
    #     output='screen'
    # ),
    Node(
        package='sensor_fusion_pkg',
        executable='gpsnode',
        name='gps_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'gps_node:=error'],
        parameters=[{'use_sim_time': False}]
    ),
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_odom_config],
        remappings=[
            ('odometry/filtered','odometry/local')
            ]
    ),
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_map_config],
        remappings=[
            ('odometry/filtered','odometry/global')
            ]
    ),
    Node(
        package='sensor_fusion_pkg',
        executable='imu_rpy_node',
        name='imu_rpy_node',
        output='screen',
        parameters=[{'use_sim_time': False}]    
    ),
    Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config,{'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'navsat_transform_node:=error'],
        remappings=[
            ('/imu','/imu'),
            ('gps/fix', 'gps/fix'),
            ('gps/filtered','gps/filtered'),            
            ('odometry/filtered', '/odometry/global')
        ]
    )
])
