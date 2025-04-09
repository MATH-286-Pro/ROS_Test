# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import serial

# class CanSerialWriteNode(Node):
#     def __init__(self):
#         super().__init__('can_serial_write_node')

#         # 打开串口。假设这是另一个 USB2CAN 设备 /dev/ttyACM0
#         try:
#             self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#             self.get_logger().info("Serial port opened successfully for sending.")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             self.ser = None
#             return

#         # 订阅“forward_can”话题，获取要发送的 CAN 帧
#         self.subscription = self.create_subscription(
#             String,
#             'forward_can',
#             self.send_can_callback,
#             10
#         )
#         self.get_logger().info("CanSerialWriteNode subscribed to 'forward_can' topic.")

#     def send_can_callback(self, msg: String):
#         """
#         假设 msg.data 格式: "0x123 8 0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88"
#         其中:
#         - 0x123 表示十六进制的 CAN ID
#         - 8     表示 DLC=8
#         - 后面八个字节 (都带0x前缀)
#         """
#         if not self.ser:
#             return

#         parts = msg.data.split()
#         if len(parts) < 2:
#             self.get_logger().warn(f"Received malformed forward_can message: {msg.data}")
#             return

#         try:
#             can_id = int(parts[0], 0)   # 识别0x前缀的十六进制 或 十进制
#             dlc = int(parts[1])
#             if dlc < 0 or dlc > 8:
#                 self.get_logger().warn("DLC out of range (0-8).")
#                 return

#             # 载荷
#             payload_data = []
#             if dlc > 0:
#                 if len(parts) < 2 + dlc:
#                     self.get_logger().warn("Not enough bytes for payload.")
#                     return
#                 payload_data = [int(x, 0) for x in parts[2:2 + dlc]]

#             # 构造下行串口指令, 示例协议: AA 11 DLC ID(4字节小端) + payload
#             frame = bytearray()
#             frame.append(0x55) #00FF00 AA
#             frame.append(0xAA) #00FF00 11
#             frame.append(dlc & 0xFF)
#             frame.extend(can_id.to_bytes(4, byteorder='little', signed=False))
#             frame.extend(payload_data)

#             # 若固件需要固定 16 字节, 可在此补齐:
#             # frame += b'\x00' * (16 - len(frame))

#             # 发送
#             self.ser.write(frame)

#             self.get_logger().info(
#                 f"Sent CAN frame: ID=0x{can_id:X}, DLC={dlc}, Payload={[hex(b) for b in payload_data]}"
#             )

#         except ValueError as e:
#             self.get_logger().warn(f"Invalid input for CAN send: {msg.data}, error={e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = CanSerialWriteNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CanSerialWriteNode(Node):
    def __init__(self):
        super().__init__('can_serial_write_node')

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
            'forward_can',
            self.send_can_callback,
            10
        )
        self.get_logger().info("CanSerialWriteNode subscribed to 'forward_can' topic.")

    def send_can_callback(self, msg: String):
        """
        假设 msg.data 格式: "0x123 8 0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88"
        其中:
        - 0x123 表示十六进制的 CAN ID
        - 8     表示 DLC=8
        - 后面八个字节 (都带0x前缀)
        """
        if not self.ser:
            return

        parts = msg.data.split()
        if len(parts) < 2:
            self.get_logger().warn(f"Received malformed forward_can message: {msg.data}")
            return

        try:
            can_id = int(parts[0], 0)   # 识别0x前缀的十六进制或十进制
            dlc = int(parts[1])
            if not (0 <= dlc <= 8):
                self.get_logger().warn("DLC out of range (0-8).")
                return

            # 载荷
            payload_data = []
            if dlc > 0:
                if len(parts) < 2 + dlc:
                    self.get_logger().warn("Not enough bytes for payload.")
                    return
                payload_data = [int(x, 0) for x in parts[2:2 + dlc]]

            #------------------------------------------------------------------
            #  组装固定 30 字节协议:
            #  Byte offset | 含义               |  示例填充
            #  ------------|--------------------|------------------------
            #      0       | 帧头(固定0x55)     | 0x55
            #      1       | 帧头(固定0xAA)     | 0xAA
            #      2       | 帧长(0x1E=30字节)  | 0x1E
            #      3       | 命令(0x01)         | 0x01
            #      4~7     | 发送次数          | 全部填0
            #      8~11    | 时间间隔          | 全部填0
            #      12      | ID类型(0=标准帧1=扩展帧) | 根据can_id判断
            #      13~16   | CAN ID(小端)       | 
            #      17      | 帧类型(0=数据帧)   | 0
            #      18      | DLC(0~8)          |
            #      19      | idAcc             | 0
            #      20      | dataAcc           | 0
            #      21~28   | data[8字节]       | 不足8字节就补0
            #      29      | CRC(暂时填0)       | 0
            #------------------------------------------------------------------

            frame = bytearray(30)

            # 帧头
            frame[0] = 0x55
            frame[1] = 0xAA

            # 帧长(固定30字节, 即0x1E)
            frame[2] = 0x1E

            # 命令(0x01 -> 转发CAN数据帧)
            frame[3] = 0x01

            # 发送次数(4字节), 这里都置0
            frame[4:8] = (0).to_bytes(4, byteorder='little')

            # 时间间隔(4字节), 这里都置0
            frame[8:12] = (0).to_bytes(4, byteorder='little')

            # 根据 can_id 大小判断标准/扩展帧
            if can_id > 0x7FF:
                frame[12] = 1  # 扩展帧
            else:
                frame[12] = 0  # 标准帧

            # CAN ID (4字节,小端)
            frame[13:17] = can_id.to_bytes(4, byteorder='little')

            # 帧类型(0=数据帧,1=远程帧), 示例固定发数据帧
            frame[17] = 0

            # DLC
            frame[18] = dlc

            # idAcc, dataAcc 先不使用, 置0
            frame[19] = 0
            frame[20] = 0

            # 传输 data (最多 8 字节), 不足补0
            for i in range(dlc):
                frame[21 + i] = payload_data[i]

            # CRC 暂时随意，这里置0
            frame[29] = 0

            # 发送
            self.ser.write(frame)

            self.get_logger().info(
                f"Sent CAN frame (cmd=0x01) -> ID=0x{can_id:X}, DLC={dlc}, Payload={[hex(b) for b in payload_data]}"
            )

        except ValueError as e:
            self.get_logger().warn(f"Invalid input for CAN send: {msg.data}, error={e}")


def main(args=None):
    rclpy.init(args=args)
    node = CanSerialWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
