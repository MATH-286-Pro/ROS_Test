import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}] [{name}]: {message}"


import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import re

class GM6020SignalAnalysisNode(Node):
    def __init__(self):
        super().__init__('gm6020_signal_analysis_node')
        
        # 订阅 "can_data" 话题，接收形如 "CAN ID: 205, DATA: 12 34 56 78 9A BC DE F0" 的字符串
        self.subscription = self.create_subscription(
            String,
            'can_data',
            self.can_data_callback,
            10
        )

        # 新建一个发布器用于发送电机角度信息
        self.angle_publisher = self.create_publisher(Float32, 'motor_angle', 10)

        self.get_logger().info("GM6020 Signal Analysis Node has been started.")

    def can_data_callback(self, msg):
        """
        回调函数：解析 can_data 话题中的字符串，提取 CAN ID 和原始 8 字节数据，
        然后根据 GM6020 协议格式进行信号解码并打印。
        """
        data_str = msg.data.strip()
        # 通常发布的格式如: "CAN ID: 205, DATA: 12 34 56 78 9A BC DE F0"
        # 可以使用正则来分割出 can_id 与 payload_str
        match = re.search(r"CAN ID:\s*([0-9A-Fa-f]+),\s*DATA:\s*(.*)", data_str)
        if not match:
            return  # 不符合预期格式，不做处理

        can_id_str  = match.group(1)          # 如 "205"
        payload_str = match.group(2).strip()  # 如 "12 34 56 78 9A BC DE F0"

        # 转为整型
        can_id = int(can_id_str, 16) - 0x204  # 十六进制字符串转 int

        # 解析 payload 中的 8 个字节
        bytes_str_list = payload_str.split()
        if len(bytes_str_list) < 8:
            return  # 数据不足 8 字节，不处理

        try:
            payload = [int(b, 16) for b in bytes_str_list[:8]]
        except ValueError:
            return  # 出现无法转为 16 进制数的情况

        # GM6020 数据解析
        # 假设：DATA[0,1] = 机械角度, 
        #      DATA[2,3] = 转速, 
        #      DATA[4,5] = 实际转矩, 
        #      DATA[6]   = 温度
        angle  = (payload[0] << 8) | payload[1]    # 机械角度 0~8191, 也可能是 0~65535 范围
        speed  = (payload[2] << 8) | payload[3]    # 转速
        torque = (payload[4] << 8) | payload[5]    # 实际转矩或电流
        temperature = payload[6]                   # 温度

        # Speed 和 Torque 有正负号，需要转换
        # 对 speed 做有符号转换（16 位补码）
        if speed & 0x8000:  # 如果最高位为 1，则为负数
            speed -= 65536

        # 对 torque 做有符号转换（16 位补码）
        if torque & 0x8000:
            torque -= 65536

        # 若需要把机械角度换算到 0~360°，可自行加一行:
        angle_deg = angle * 360.0 / 8192.0

        # 根据需要，若 speed/torque 为有符号数据，可进行补码转换:
        # 例如 16 bit 有符号转:
        # if speed & 0x8000: speed -= 65536
        # if torque & 0x8000: torque -= 65536

        # 打印解析结果
        self.get_logger().info(
            f"GM6020 ID 0x{can_id:X} -> "
            f"Angle: {angle_deg:.2f}, Speed: {speed}, Torque: {torque}, "
            f"Temp: {temperature}"
        )

        # 发布电机角度消息
        from std_msgs.msg import Float32
        angle_msg = Float32()
        angle_msg.data = angle_deg
        self.angle_publisher.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GM6020SignalAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GM6020SignalAnalysisNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
