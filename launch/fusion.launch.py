from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('sensor_fusion_pkg'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
    Node(
        package='sensor_fusion_pkg',
        executable='mtnode',
        name='mtnode',
        output='screen',
        parameters=[
            {'device': '/dev/ttyUSB2'}, 
            {'baudrate': 115200},
            {'timeout': 0.002},
            {'initial_wait': 0.1},
            {'use_sim_time': False}
        ]
    ),
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
        parameters=[config_path],
        remapping=[('odometry/filtered','odometry/local')]
    ),
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[config_path],
        remapping=[('odometry/filtered','odometry/global')]
    ),
    Node(
        package='sensor_fusion_pkg',
        executable='imu_cov_node',
        name='imu_cov_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    ),
    Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_path,{'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'navsat_transform_node:=error'],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/fix'),
            ('gps/filtered','gps/filtered')
            ('odometry/gps', '/odometry/gps'),
            ('odometry/filtered', '/odometry/global')
        ]
    )
])
