from setuptools import find_packages, setup

package_name = 'sensor_fusion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['sensor_fusion_pkg']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/sensor_fusion_pkg/launch', ['launch/fusion.launch.py']),
        ('share/sensor_fusion_pkg/config', [
            'config/ekf_odom.yaml',
            'config/ekf_map.yaml',
            'config/navsat.yaml'
            ]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juwon',
    maintainer_email='juwon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'mtnode = sensor_fusion_pkg.mtnode:main',
        'gpsnode = sensor_fusion_pkg.gpsnode:main',
        'imu_rpy_node = sensor_fusion_pkg.imu_rpy_node:main',
        'odom_publisher_node = sensor_fusion_pkg.odom_publisher_node:main'
        ],
    },
)

