import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("CAN_Control")
        self.pub   = self.create_publisher(String, 'CAN_Control_Data', 10)    # send topic
        self.timer = self.create_timer(0.1, self.timer_callback)         # 0.1秒发布一次

    def timer_callback(self):
        # 假设要往 0x1FF 发送 8 字节
        can_id = 0x1FF
        dlc = 8

        # 示例：四个目标值
        val5 = 1000   # motor 5
        val6 = 2000   # motor 6
        val7 = -200   # motor 7
        val8 = 0      # motor 8

        # 大端模式 (High Byte 在前)
        def to_big_endian_bytes(value: int):
            high = (value >> 8) & 0xFF
            low  = value & 0xFF
            return high, low

        v5h, v5l = to_big_endian_bytes(val5)
        v6h, v6l = to_big_endian_bytes(val6)
        v7h, v7l = to_big_endian_bytes(val7)
        v8h, v8l = to_big_endian_bytes(val8)

        # 组装成字符串
        # 控制命令 ID 5->8 的电机
        # "0x1FF 8 0x03 0xE8 0x07 0xD0 0xFC 0x18 0x00 0x00"
        can_5_to_8 = (
            f"0x{can_id:X} {dlc} "
            f"0x{v5h:02X} 0x{v5l:02X} "
            f"0x{v6h:02X} 0x{v6l:02X} "
            f"0x{v7h:02X} 0x{v7l:02X} "
            f"0x{v8h:02X} 0x{v8l:02X}"
        )

        # 发布 can_msg 数据
        can_msg      = String()
        can_msg.data = can_5_to_8
        self.pub.publish(can_msg)

        # 打印数据
        # self.get_logger().info(f"Publish motor control: {can_5_to_8}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
