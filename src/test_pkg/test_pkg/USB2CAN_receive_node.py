import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading
import queue

FRAME_HEAD = bytes([0xAA, 0x11])
FRAME_MIN_LEN = 16
HZ = 1

# 定义类
class CanSerialNode(Node):
    def __init__(self, reading_frequency = 1000):
        super().__init__('can_serial_node')
        self.publisher_         = self.create_publisher(String, 'can_data', 10)
        self.forward_publisher_ = self.create_publisher(String, 'forward_can', 10)
        
        # 创建一个队列用于存储完整的 CAN 帧
        self.frame_queue = queue.Queue(maxsize=1000)

        # 其他初始化
        self.frame_count = 0
        self.reading_time_gap = 1 / reading_frequency
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.001)
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
        self.read_thread.start()

        # ROS 定时器定时取队列中数据进行处理
        # self.timer = self.create_timer(self.reading_time_gap, self.process_frames(print_interval=50))  # 提高处理频率
        self.timer = self.create_timer(self.reading_time_gap, lambda: self.process_frames(print_interval=50))


    def serial_read_thread(self):
        """独立线程，持续读取串口数据，并将完整帧存入队列"""
        buffer = b""
        while rclpy.ok():
            # 读取一定数量字节填充到缓冲区
            data = self.ser.read(32)
            if data:
                buffer += data

            # 检查缓冲区中是否存在完整的帧
            while len(buffer) >= FRAME_MIN_LEN:
                head_index = buffer.find(FRAME_HEAD)
                if head_index == -1:
                    # 没有找到帧头，清空无效数据
                    buffer = b""
                    break
                if len(buffer) - head_index < FRAME_MIN_LEN:
                    # 帧未完成，等待更多数据
                    break
                # 提取完整帧
                potential_frame = buffer[head_index:head_index + FRAME_MIN_LEN]
                # 根据需要可以做进一步校验，比如数据完整性检查
                # 如果校验通过，将帧加入队列，并从缓冲区中移除这部分数据
                try:
                    self.frame_queue.put(potential_frame, block=False)
                except queue.Full:
                    self.get_logger().warn("Frame queue is full. Dropping frame.")
                buffer = buffer[head_index + FRAME_MIN_LEN:]
    
    def process_frames(self, print_interval = 50):
        """在 ROS 定时器回调中处理队列中的数据帧"""
        while not self.frame_queue.empty():
            data = self.frame_queue.get()
            if data and len(data) >= FRAME_MIN_LEN:
                dlc = data[2]
                can_id = int.from_bytes(data[3:7], byteorder='little')
                payload = data[7:7+dlc]

                # 构造日志字符串
                msg_str = f"CAN ID: {can_id:X}, DATA: {' '.join(f'{b:02X}' for b in payload)}"

                # 每 print_interval 帧打印一次，减轻 I/O 压力
                self.frame_count += 1
                if self.frame_count % print_interval == 0:
                    self.get_logger().info(msg_str)

                # 发布到 can_data 话题
                ros_msg = String()
                ros_msg.data = msg_str
                self.publisher_.publish(ros_msg)

                # 发布到 forward_can 话题
                forward_str = f"0x{can_id:X} {dlc}" + ''.join(f" 0x{b:02X}" for b in payload)
                forward_msg = String()
                forward_msg.data = forward_str
                self.forward_publisher_.publish(forward_msg)


# 主函数
def main(args=None):
    rclpy.init(args=args)
    node = CanSerialNode(reading_frequency=1000*HZ)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down can_serial_node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
