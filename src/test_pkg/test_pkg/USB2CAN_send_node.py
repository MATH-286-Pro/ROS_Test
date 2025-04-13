import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "{message}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CanSerialWriteNode(Node):
    def __init__(self):
        super().__init__('USB2CAN_send')

        # 打开串口。假设这是另一个 USB2CAN 设备 /dev/ttyACM0
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully for sending.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None
            return

        # 订阅“forward_can”话题，获取要发送的 CAN 帧
        self.subscription = self.create_subscription(
            String,
            'CAN_GM6020_control',
            self.send_can_callback,
            10
        )
        self.get_logger().info("CanSerialWriteNode subscribed to 'CAN_GM6020_control' topic.")

    def send_can_callback(self, can_msg: String):
        """
        接收的消息格式必须为:
          "0x123 8 0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88"
        其中:
          - 第一个字段 0x123 为 CAN ID (可以是 16 进制或十进制)
          - 第二个字段必须为 8，代表后面有 8 个数据字节
          - 后续 8 个字段各代表一字节数据
        如果 DLC 不为 8，直接忽略该消息，避免后续数据冲突。
        """
        if not self.ser:
            return

        parts = can_msg.data.split()
        if len(parts) < 2:
            self.get_logger().warn(f"Received malformed can_GM6020 message: {can_msg.data}")
            return

        try:
            can_id = int(parts[0], 0)   # 支持0x前缀的十六进制或十进制
            dlc    = int(parts[1])
            
            # 仅允许接收8字节数据的CAN信号
            if dlc != 8:
                self.get_logger().warn(f"Received message with DLC {dlc}; only messages with DLC = 8 are accepted.")
                return

            # 检查是否有足够的payload数据
            if len(parts) < 2 + dlc:
                self.get_logger().warn("Not enough bytes for payload.")
                return

            payload_data = [int(x, 0) for x in parts[2:2 + dlc]]

            #------------------------------------------------------------------
            # 组装固定 30 字节协议:
            # Byte offset | 含义                        | 说明
            # ------------|-----------------------------|-----------------------------------------
            #      0      | 帧头(固定0x55)              | 0x55
            #      1      | 帧头(固定0xAA)              | 0xAA
            #      2      | 帧长(0x1E=30字节)           | 0x1E
            #      3      | 命令(0x01)                  | 0x01 (转发CAN数据帧)
            #      4~7    | 发送次数                    | 全部填0
            #      8~11   | 时间间隔                    | 全部填0
            #      12     | ID类型(0=标准帧, 1=扩展帧)   | 根据can_id判断
            #      13~16  | CAN ID(小端)                |
            #      17     | 帧类型(0=数据帧)            | 0
            #      18     | DLC                         | 此处固定为8
            #      19     | idAcc                       | 0
            #      20     | dataAcc                     | 0
            #      21~28  | data[8字节]                 | 如不足8字节会被补0
            #      29     | CRC(暂时填0)                | 0
            #------------------------------------------------------------------

            frame = bytearray(30)

            # 帧头 & 帧长 & 命令
            frame[0] = 0x55
            frame[1] = 0xAA
            frame[2] = 0x1E
            frame[3] = 0x01

            # 发送次数(4字节) & 时间间隔(4字节)，均置0
            frame[4:8]  = (0).to_bytes(4, byteorder='little')
            frame[8:12] = (0).to_bytes(4, byteorder='little')

            # 根据can_id判断标准帧或扩展帧 (0x7FF以内为标准帧)
            frame[12] = 1 if can_id > 0x7FF else 0

            # CAN ID (4字节, 小端存储)
            frame[13:17] = can_id.to_bytes(4, byteorder='little')

            # 帧类型 (0=数据帧)
            frame[17] = 0

            # DLC 固定为 8
            frame[18] = dlc

            # idAcc, dataAcc 均置0
            frame[19] = 0
            frame[20] = 0

            # 填充 payload (最多8字节, 如果不足8字节不会出现在这里，因为dlc必须为8)
            for i in range(dlc):
                frame[21 + i] = payload_data[i]

            # CRC 暂时填0
            frame[29] = 0

            # 发送帧数据
            self.ser.write(frame)

            self.get_logger().info(
                f"Sent CAN frame (cmd=0x01) -> ID=0x{can_id:X}, DLC={dlc}, Payload={[hex(b) for b in payload_data]}"
            )

        except ValueError as e:
            self.get_logger().warn(f"Invalid input for CAN send: {can_msg.data}, error={e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanSerialWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
