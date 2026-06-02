// Copyright 2026 zhouyi
// License: Apache-2.0

#ifndef TUNNEL_CENTERLINE_EXTRACTOR__TUNNEL_CENTERLINE_NODE_HPP_
#define TUNNEL_CENTERLINE_EXTRACTOR__TUNNEL_CENTERLINE_NODE_HPP_

#include "tunnel_centerline_extractor/grid_types.hpp"
#include "tunnel_centerline_extractor/distance_field.hpp"
#include "tunnel_centerline_extractor/skeleton_extractor.hpp"
#include "tunnel_centerline_extractor/centerline_graph.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace tunnel_centerline_extractor
{

class TunnelCenterlineNode : public rclcpp::Node
{
public:
  TunnelCenterlineNode();

private:
  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publish_results(const Grid2D & free_mask, const Grid2D & distance,
                       const Grid2D & skeleton, const CenterlineGraph & graph);
  void publish_markers(const Grid2D & skeleton, const CenterlineGraph & graph);

  // ── parameters ───────────────────────────────────────────
  std::string map_topic_;
  bool publish_distance_map_;
  bool publish_risk_map_;
  bool publish_markers_;
  int free_threshold_;
  int occupied_threshold_;
  double min_centerline_distance_m_;
  int skeleton_prune_length_cells_;
  int branch_merge_radius_cells_;
  int endpoint_merge_radius_cells_;
  double safe_distance_m_;
  bool unknown_is_blocked_;
  double publish_period_seconds_;

  // ── subscriptions / publishers ───────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dist_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr risk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ── cached latest results ────────────────────────────────
  Grid2D latest_distance_;
  Grid2D latest_risk_;
  Grid2D latest_skeleton_;
  CenterlineGraph latest_graph_;
  bool has_results_ = false;
};

}  // namespace tunnel_centerline_extractor

#endif  // TUNNEL_CENTERLINE_EXTRACTOR__TUNNEL_CENTERLINE_NODE_HPP_
