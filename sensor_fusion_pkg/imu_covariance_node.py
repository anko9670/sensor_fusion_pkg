import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceNode(Node):
    def __init__(self):
        super().__init__('imu_covariance_node')

        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

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
    node = ImuCovarianceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
