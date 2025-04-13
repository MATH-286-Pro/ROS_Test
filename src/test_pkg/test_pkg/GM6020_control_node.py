import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("GM6020_Control")
        self.pub   = self.create_publisher(String, 'CAN_GM6020_control', 10)    # send topic
        self.timer = self.create_timer(0.1, self.timer_callback)         # 0.1秒发布一次

    def timer_callback(self):
        # 假设要往 0x1FF 发送 8 字节
        can_id = 0x1FF
        dlc = 8

        # 示例：四个目标值
        val1 = 1000   # motor1
        val2 = 2000   # motor2
        val3 = -1000  # motor3
        val4 = 0      # motor4

        # 大端模式 (High Byte 在前)
        def to_big_endian_bytes(value: int):
            high = (value >> 8) & 0xFF
            low  = value & 0xFF
            return high, low

        v1h, v1l = to_big_endian_bytes(val1)
        v2h, v2l = to_big_endian_bytes(val2)
        v3h, v3l = to_big_endian_bytes(val3)
        v4h, v4l = to_big_endian_bytes(val4)

        # 组装成字符串
        # "0x1FF 8 0x03 0xE8 0x07 0xD0 0xFC 0x18 0x00 0x00"
        can_GM6020 = (
            f"0x{can_id:X} {dlc} "
            f"0x{v1h:02X} 0x{v1l:02X} "
            f"0x{v2h:02X} 0x{v2l:02X} "
            f"0x{v3h:02X} 0x{v3l:02X} "
            f"0x{v4h:02X} 0x{v4l:02X}"
        )

        # 发布 can_msg 数据
        can_msg      = String()
        can_msg.data = can_GM6020
        self.pub.publish(can_msg)

        self.get_logger().info(f"Publish motor control: {can_GM6020}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
