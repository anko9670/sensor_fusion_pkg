# zed_f9p_rtk_node.py
import rclpy
from rclpy.node import Node
import subprocess

class ZEDF9PRTKNode(Node):
    def __init__(self):
        super().__init__('zed_f9p_rtk_node')

        # RTCM 보정신호 수신 → ZED-F9P로 전달
        rtcm_command = (
            "/usr/local/bin/str2str "
            "-in ntrip://seoul:seoul@gnss.eseoul.go.kr:2101/DBON-RTCM32 "
            "-out serial://ttyACM0:115200"
        )
        self.get_logger().info("Sending RTCM correction to ZED-F9P via /dev/ttyACM0...")
        self.rtk_proc = subprocess.Popen(rtcm_command, shell=True)

        # ublox_gps 노드 실행
        gps_command = (
                "ros2 run ublox_gps ublox_gps_node "
                "--ros-args "
                "-p port:=/dev/ttyACM0 "
                "-p frame_id:=gps "
                "--params-file ~/ros2_ws/src/ublox/ublox_gps/config/zed_f9p.yaml "
                "--remap /ublox_gps_node/fix:=/gps/fix"
        )
        self.get_logger().info("Starting ublox_gps_node...")
        self.gps_proc = subprocess.Popen(gps_command, shell=True)

    def destroy_node(self):
        self.get_logger().info("Shutting down GPS and RTK processes...")
        if self.rtk_proc:
            self.rtk_proc.terminate()
        if self.gps_proc:
            self.gps_proc.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDF9PRTKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()