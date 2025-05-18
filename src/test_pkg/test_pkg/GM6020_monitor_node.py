import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import re
import time

class GM6020SignalAnalysisNode(Node):
    def __init__(self):

        # 节点初始化 + 节点名称
        super().__init__('GM6020_monitor') 
        
        # 订阅 "can_data" 话题，接收形如 "CAN ID: 205, DATA: 12 34 56 78 9A BC DE F0" 的字符串
        self.subscription = self.create_subscription(
            String,
            'CAN_GM6020_feedback',
            self.can_data_callback,
            10
        )

        # 发布 "motor_angle" 话题
        self.angle_publisher = self.create_publisher(Float32, 'motor_angle', 10)

        # 限制发布频率，避免数据太高导致 rqt_plot 卡顿
        publish_fps = 25
        self.last_publish_time = time.time()
        self.publish_interval  = 1/publish_fps  # 每 xxx 秒发布一次，10Hz

        # 打印初始化结束信息
        self.get_logger().info("Start GM6020 monitor node.")

    def can_data_callback(self, msg):
        """
        回调函数：解析 can_data 话题中的字符串，提取 CAN ID 和原始 8 字节数据，
        然后根据 GM6020 协议格式进行信号解码、打印解析结果并发布电机角度数据。
        """
        data_str = msg.data.strip()

        # 正则提取 "CAN ID" 和 "DATA" 部分
        match = re.search(r"CAN ID:\s*([0-9A-Fa-f]+),\s*DATA:\s*(.*)", data_str)
        if not match:
            return  # 格式不匹配则不处理

        can_id_str  = match.group(1)            # 例如 "205"
        payload_str = match.group(2).strip()    # 例如 "12 34 56 78 9A BC DE F0"

        # ID 过滤
        target_can_id = 0x206  
        if int(can_id_str, 16) != target_can_id:
            return

        # 将 CAN ID 字符串转为整型，减去偏移值 0x204
        can_id = int(can_id_str, 16) - 0x204

        # 分割并转换 payload 中的字节
        bytes_str_list = payload_str.split()
        if len(bytes_str_list) < 8:
            return  # 数据不足 8 字节时跳过处理

        try:
            payload = [int(b, 16) for b in bytes_str_list[:8]]
        except ValueError:
            return  # 出现无法转换的情况则跳过处理

        # GM6020 数据解析:
        # DATA[0,1] = 机械角度, DATA[2,3] = 转速, DATA[4,5] = 实际转矩, DATA[6] = 温度
        angle  = (payload[0] << 8) | payload[1]
        speed  = (payload[2] << 8) | payload[3]
        torque = (payload[4] << 8) | payload[5]
        temperature = payload[6]

        # 对转速和转矩做 16 位有符号数转换
        if speed & 0x8000:
            speed -= 65536
        if torque & 0x8000:
            torque -= 65536

        # 将机械角度转换为角度制（假设 0~8191 对应 0~360°）
        angle_deg = angle * 360.0 / 8192.0

        # 打印解析结果
        self.get_logger().debug(
            f"GM6020 ID 0x{can_id:X} -> "
            f"Angle: {angle_deg:.2f}, Speed: {speed}, Torque: {torque}, "
            f"Temp: {temperature}"
        )

        # 限制发布频率：若超过设定的时间间隔，则发布角度消息
        current_time = time.time()
        if current_time - self.last_publish_time >= self.publish_interval:
            angle_msg = Float32()
            angle_msg.data = angle_deg
            self.angle_publisher.publish(angle_msg)
            self.last_publish_time = current_time

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
