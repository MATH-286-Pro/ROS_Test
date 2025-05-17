colcon build --symlink-install
source install/setup.bash
ros2 launch test_pkg demo.launch.py
# ros2 run test_pkg USB2CAN_receive_node