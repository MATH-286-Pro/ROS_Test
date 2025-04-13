#!/usr/bin/env python3
import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        # 创建一个发布者，话题名称为 keyboard_input，消息类型为 std_msgs/String
        self.publisher = self.create_publisher(String, 'keyboard_input', 10)
        # 保存终端原始配置，便于恢复
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        # 将终端设置为原始模式
        tty.setraw(sys.stdin.fileno())
        # 首先等待是否有输入
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = ''
        if rlist:
            # 读取第一个字符
            key = sys.stdin.read(1)
            # 如果第一个字符为转义字符，则可能是一个特殊按键（比如方向键）
            if key == '\x1b':
                # 检查是否还有后续字符
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    key += sys.stdin.read(1)
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    key += sys.stdin.read(1)
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def run(self):
        self.get_logger().info("键盘检测节点启动，请按键...")
        while rclpy.ok():
            key = self.get_key()
            if key:
                msg = String()
                msg.data = key
                # 发布消息
                self.publisher.publish(msg)
                self.get_logger().info(f'发布消息: "{key}"')
                # 如果检测到 Ctrl+C（通常对应于 '\x03'），退出循环
                if key == '\x03':
                    break

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"异常: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
