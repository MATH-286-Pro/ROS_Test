import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("DJI_Motor_Control")
        self.pub   = self.create_publisher(String, 'CAN_DJI_Motor_Control', 10)     # 发布 CAN_DJI_Motor_Control 信息
        self.timer = self.create_timer(0.1, self.timer_callback)                    # 0.1秒发布一次

    def timer_callback(self):

        # DJI 电机 ID
        can_id = 0x1FF
        dlc = 8

        # 示例：四个目标值
        motor5_torque = 0      # motor ID = 5
        motor6_torque = 2000   # motor ID = 6
        motor7_torque = 300    # motor ID = 7
        motor8_torque = 0      # motor ID = 8

        # 大端模式 (High Byte 在前)
        def to_big_endian_bytes(value: int):
            high = (value >> 8) & 0xFF
            low  = value & 0xFF
            return high, low

        v1h, v1l = to_big_endian_bytes(motor5_torque)
        v2h, v2l = to_big_endian_bytes(motor6_torque)
        v3h, v3l = to_big_endian_bytes(motor7_torque)
        v4h, v4l = to_big_endian_bytes(motor8_torque)

        # 组装成字符串
        # "0x1FF 8 0x03 0xE8 0x07 0xD0 0xFC 0x18 0x00 0x00"
        CAN_DATA_0x1FF = (
            f"0x{can_id:X} {dlc} "
            f"0x{v1h:02X} 0x{v1l:02X} "
            f"0x{v2h:02X} 0x{v2l:02X} "
            f"0x{v3h:02X} 0x{v3l:02X} "
            f"0x{v4h:02X} 0x{v4l:02X}"
        )

        # 发布 can_msg 数据
        can_msg      = String()
        can_msg.data = CAN_DATA_0x1FF
        self.pub.publish(can_msg)

        # 数据打印
        self.get_logger().debug(f"Publish motor control: {CAN_DATA_0x1FF}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()