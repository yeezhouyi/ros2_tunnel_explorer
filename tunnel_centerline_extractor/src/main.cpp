// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/tunnel_centerline_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tunnel_centerline_extractor::TunnelCenterlineNode>());
  rclcpp::shutdown();
  return 0;
}
