# colcon build --symlink-install
# source install/setup.bash
# ros2 launch test_pkg demo.launch.py


# # 测试 demo_cpp_pkg
# colcon build --packages-select demo_cpp_pkg
# source install/setup.bash
# ros2 pkg executables demo_cpp_pkg
# ros2 run demo_cpp_pkg demo_cpp_node


# 测试 rm_control_cpp_pkg
colcon build --packages-select rm_control_cpp_pkg
source install/setup.bash
ros2 run rm_control_cpp_pkg USB2CAN_node