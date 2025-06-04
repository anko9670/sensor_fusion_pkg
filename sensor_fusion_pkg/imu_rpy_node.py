import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from transforms3d.euler import quat2euler, euler2quat
from math import atan2, sin, cos, radians, degrees
import numpy as np

class ImuRpyPublisher(Node):
    def __init__(self):
        super().__init__('imu_rpy_publisher')

        self.roll = 0.0
        self.pitch = 0.0
        self.has_imu = False

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu', 10)

        self.get_logger().info("IMU orientation corrected publisher started (EKF-compatible).")

    def mag_callback(self, msg: MagneticField):
        if not self.has_imu:
            return

        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        # Tilt 보정된 자기장 기반 Yaw 계산
        Xh = mx * cos(self.pitch) + mz * sin(self.pitch)
        Yh = mx * sin(self.roll) * sin(self.pitch) + my * cos(self.roll) - mz * sin(self.roll) * cos(self.pitch)
        yaw_rad = atan2(Yh, Xh)

        # 쿼터니언 변환 (rad 단위)
        quat = euler2quat(self.roll, self.pitch, yaw_rad, axes='sxyz')

        # Imu 메시지 생성
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'  # 필요 시 수정

        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

    def imu_callback(self, msg):
        # orientation_covariance 설정
        msg.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]
        # angular_velocity_covariance 설정
        msg.angular_velocity_covariance = [
            0.1, 0.0,  0.0,
            0.0, 0.1,  0.0,
            0.0, 0.0,  0.1
        ]
        # linear_acceleration_covariance 설정
        msg.linear_acceleration_covariance = [
            0.1, 0.0,  0.0,
            0.0, 0.1,  0.0,
            0.0, 0.0,  0.1
        ]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRpyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
