#include "rclcpp/rclcpp.hpp"





int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("usb2can_node");
  RCLCPP_INFO(node->get_logger(), "Start USB2CAN Node");
  rclcpp::spin(node);
  rclcpp::shutdown();


  return 0;
}