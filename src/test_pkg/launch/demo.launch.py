import launch
from launch_ros.actions import Node

def generate_launch_description():

    USB2CAN_send_node = Node(
        package = 'test_pkg',           # 包名称
        executable='USB2CAN_send_node',
        # output = 'screen'               # 输出 log, screen, both
    )

    USB2CAN_receive_node = Node(
        package = 'test_pkg',
        executable='USB2CAN_receive_node',
        # output = 'screen'
    )

    GM6020_control_node = Node(
        package = 'test_pkg', 
        executable='GM6020_control_node',
        # output = 'screen'
    )

    GM6020_monitor_node = Node(
        package = 'test_pkg', 
        executable='GM6020_monitor_node',
        # output = 'screen'
    )

    return launch.LaunchDescription([
        # Actions
        USB2CAN_send_node,
        USB2CAN_receive_node,
        GM6020_control_node,
        GM6020_monitor_node,
    ])