import rclpy
from rclpy.node import Node
import subprocess
import time

class RTKSinglePortNode(Node):
    def __init__(self):
        super().__init__('rtk_single_port_node')

        # Step 1: str2strë¡œ RTCM ë³´ì • ì „ì†¡ ì‹œìž‘ (ê³„ì† ì‹¤í–‰)
        rtcm_command = (
            "/usr/local/bin/str2str "
            "-in ntrip://seoul:seoul@gnss.eseoul.go.kr:2101/DBON-RTCM32 "
            "-out serial://ttyUSB0:115200"
        )
        self.get_logger().info("Sending RTCM correction to LC29H...")
        self.rtk_proc = subprocess.Popen(rtcm_command, shell=True)

        # Step 2: nmea_serial_driverë¡œ GPS ìˆ˜ì‹  ì‹œìž‘ (ë…ë¦½ì ìœ¼ë¡œ ì‹¤í–‰)
        gps_command = (
            "ros2 run nmea_navsat_driver nmea_serial_driver "
            "--ros-args -p port:=/dev/ttyUSB0 -p baud:=115200"
        )
        self.get_logger().info("Starting GPS data node...")
        self.gps_proc = subprocess.Popen(gps_command, shell=True)

    def destroy_node(self):
        self.get_logger().info("Shutting down GPS and RTK processes...")
        if self.rtk_proc:
            self.rtk_proc.terminate()  # RTCM ë³´ì • ì „ì†¡ ì¢…ë£Œ
        if self.gps_proc:
            self.gps_proc.terminate()  # GPS ë°ì´í„° ìˆ˜ì‹  ì¢…ë£Œ
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RTKSinglePortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()