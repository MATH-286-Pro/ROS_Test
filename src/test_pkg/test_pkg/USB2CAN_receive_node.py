import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CanSerialNode(Node):
    def __init__(self):
        super().__init__('can_serial_node')
        self.publisher_ = self.create_publisher(String, 'can_data', 10)
        # 新增一个用于“转发”的Publisher
        self.forward_publisher_ = self.create_publisher(String, 'forward_can', 10)

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz

    def read_serial(self):
        data = self.ser.read(16)
        if len(data) == 16 and data[0] == 0xAA and data[1] == 0x11:
            dlc = data[2]
            can_id = int.from_bytes(data[3:7], byteorder='little')
            payload = data[7:7+dlc]

            # 1) 在本地日志里打印 & 发布到 can_data
            msg_str = f"CAN ID: {can_id:X}, DATA: {' '.join(f'{b:02X}' for b in payload)}"
            self.get_logger().info(msg_str)

            ros_msg = String()
            ros_msg.data = msg_str
            self.publisher_.publish(ros_msg)

            # 2) 再将该 CAN 帧重新打包为 USB2CAN_send_node.py 可识别的格式
            #    假设发送节点期望的格式为: "0xCAN_ID DLC 0xbyte1 0xbyte2 ... 0xbyteN"
            forward_str = f"0x{can_id:X} {dlc}"
            for b in payload:
                forward_str += f" 0x{b:02X}"

            forward_msg = String()
            forward_msg.data = forward_str
            self.forward_publisher_.publish(forward_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CanSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
