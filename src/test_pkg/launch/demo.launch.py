import launch
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    # 设置全局日志阈值为 WARN，过滤掉 info 级别及以下的日志
    log_env = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'WARN'
    )

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
        executable='DJI_motor_control_node',
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