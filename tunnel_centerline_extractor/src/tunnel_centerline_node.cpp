// Copyright 2026 zhouyi
// License: Apache-2.0

#include "tunnel_centerline_extractor/tunnel_centerline_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace tunnel_centerline_extractor
{

TunnelCenterlineNode::TunnelCenterlineNode()
: Node("tunnel_centerline_extractor")
{
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  publish_distance_map_ = declare_parameter<bool>("publish_distance_map", true);
  publish_risk_map_ = declare_parameter<bool>("publish_risk_map", true);
  publish_markers_ = declare_parameter<bool>("publish_markers", true);
  free_threshold_ = declare_parameter<int>("free_threshold", 20);
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
  min_centerline_distance_m_ = declare_parameter<double>("min_centerline_distance_m", 0.35);
  skeleton_prune_length_cells_ = declare_parameter<int>("skeleton_prune_length_cells", 15);
  branch_merge_radius_cells_ = declare_parameter<int>("branch_merge_radius_cells", 10);
  endpoint_merge_radius_cells_ = declare_parameter<int>("endpoint_merge_radius_cells", 10);
  safe_distance_m_ = declare_parameter<double>("safe_distance_m", 1.0);
  unknown_is_blocked_ = declare_parameter<bool>("unknown_is_blocked", true);
  publish_period_seconds_ = declare_parameter<double>("publish_period_seconds", 1.0);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_callback(msg); });

  if (publish_distance_map_)
    dist_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/tunnel_centerline/distance_map", 1);
  if (publish_risk_map_)
    risk_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/tunnel_centerline/risk_map", 1);
  if (publish_markers_)
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tunnel_centerline/markers", 10);

  int period_ms = static_cast<int>(publish_period_seconds_ * 1000.0);
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    [this]() {
      if (!has_results_) return;
      publish_results(latest_distance_, latest_distance_,
                      latest_skeleton_, latest_graph_);
    });

  RCLCPP_INFO(get_logger(), "Tunnel centerline extractor started");
}

void TunnelCenterlineNode::map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  const auto & info = msg->info;

  Grid2D free_mask = build_free_mask(
    info.width, info.height, info.resolution,
    info.origin.position.x, info.origin.position.y,
    msg->data, free_threshold_, occupied_threshold_, unknown_is_blocked_);

  Grid2D distance = brushfire_distance_field(free_mask);

  // Build skeleton from free cells whose distance >= min threshold
  // (this filters out narrow kinks near walls)
  Grid2D thick_center;
  thick_center.width = free_mask.width;
  thick_center.height = free_mask.height;
  thick_center.resolution = free_mask.resolution;
  thick_center.origin_x = free_mask.origin_x;
  thick_center.origin_y = free_mask.origin_y;
  thick_center.data.resize(free_mask.width * free_mask.height, 0);

  double min_dist_units = min_centerline_distance_m_ * 1000.0;
  for (int i = 0; i < free_mask.width * free_mask.height; ++i) {
    if (free_mask.data[i] && distance.data[i] >= min_dist_units)
      thick_center.data[i] = 1;
  }

  Grid2D skeleton = extract_skeleton(thick_center, skeleton_prune_length_cells_);

  CenterlineGraph graph = extract_centerline_graph(
    skeleton, distance, info.resolution, branch_merge_radius_cells_);

  // Build risk map: risk = 1.0 - clamp(dist / safe_dist, 0, 1)
  // Occupancy: 0=low risk, 100=high risk, -1=unknown/blocked
  latest_risk_.width = free_mask.width;
  latest_risk_.height = free_mask.height;
  latest_risk_.resolution = free_mask.resolution;
  latest_risk_.origin_x = free_mask.origin_x;
  latest_risk_.origin_y = free_mask.origin_y;
  latest_risk_.data.resize(free_mask.width * free_mask.height, -1);

  for (int i = 0; i < free_mask.width * free_mask.height; ++i) {
    if (!free_mask.data[i]) { latest_risk_.data[i] = -1; continue; }
    double dist_m = distance.data[i] / 1000.0;
    double risk = 1.0 - std::min(dist_m / safe_distance_m_, 1.0);
    latest_risk_.data[i] = static_cast<int>(risk * 100.0);
  }

  // Cache results
  latest_distance_ = std::move(distance);
  latest_skeleton_ = std::move(skeleton);
  latest_graph_ = std::move(graph);
  has_results_ = true;

  // Publish immediately on first map
  publish_results(free_mask, latest_distance_, latest_skeleton_, latest_graph_);
}

void TunnelCenterlineNode::publish_results(
  const Grid2D &, const Grid2D &, const Grid2D &, const CenterlineGraph &)
{
  if (!has_results_) return;

  auto now = this->now();

  // Distance map
  if (publish_distance_map_ && dist_pub_) {
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.frame_id = "map"; msg->header.stamp = now;
    msg->info.width = latest_distance_.width;
    msg->info.height = latest_distance_.height;
    msg->info.resolution = latest_distance_.resolution;
    msg->info.origin.position.x = latest_distance_.origin_x;
    msg->info.origin.position.y = latest_distance_.origin_y;
    msg->info.origin.orientation.w = 1.0;
    msg->data.resize(latest_distance_.data.size());
    for (size_t i = 0; i < latest_distance_.data.size(); ++i) {
      // Scale to 0–100 occupancy for RViz display
      double m = latest_distance_.data[i] / 1000.0;
      msg->data[i] = static_cast<int8_t>(std::min(m * 100.0, 100.0));
    }
    dist_pub_->publish(std::move(msg));
  }

  // Risk map
  if (publish_risk_map_ && risk_pub_) {
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.frame_id = "map"; msg->header.stamp = now;
    msg->info.width = latest_risk_.width;
    msg->info.height = latest_risk_.height;
    msg->info.resolution = latest_risk_.resolution;
    msg->info.origin.position.x = latest_risk_.origin_x;
    msg->info.origin.position.y = latest_risk_.origin_y;
    msg->info.origin.orientation.w = 1.0;
    msg->data.assign(latest_risk_.data.begin(), latest_risk_.data.end());
    risk_pub_->publish(std::move(msg));
  }

  if (publish_markers_)
    publish_markers(latest_skeleton_, latest_graph_);
}

void TunnelCenterlineNode::publish_markers(
  const Grid2D & skeleton, const CenterlineGraph & graph)
{
  visualization_msgs::msg::MarkerArray arr;

  auto make_deleteall = [](const std::string & ns) {
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    m.ns = ns;
    return m;
  };

  auto now = this->now();

  // ── Skeleton centerline (cyan LINE_STRIP) ──────────────────
  if (!graph.corridor_nodes.empty() || !graph.endpoints.empty() ||
      !graph.branch_points.empty()) {
    visualization_msgs::msg::Marker skel;
    skel.header.frame_id = "map";
    skel.header.stamp = now;
    skel.ns = "centerline_skeleton";
    skel.id = 0;
    skel.type = visualization_msgs::msg::Marker::POINTS;
    skel.action = visualization_msgs::msg::Marker::ADD;
    skel.scale.x = 0.06;
    skel.scale.y = 0.06;
    skel.color.a = 0.9;
    skel.color.r = 0.0; skel.color.g = 1.0; skel.color.b = 1.0;

    for (const auto & n : graph.corridor_nodes) {
      geometry_msgs::msg::Point p;
      p.x = n.wx; p.y = n.wy; p.z = 0.02;
      skel.points.push_back(p);
    }
    for (const auto & n : graph.endpoints) {
      geometry_msgs::msg::Point p;
      p.x = n.wx; p.y = n.wy; p.z = 0.02;
      skel.points.push_back(p);
    }
    arr.markers.push_back(make_deleteall("centerline_skeleton"));
    arr.markers.push_back(skel);
  }

  // ── Branch points (red SPHERE) ─────────────────────────────
  if (!graph.branch_points.empty()) {
    visualization_msgs::msg::Marker bp;
    bp.header.frame_id = "map";
    bp.header.stamp = now;
    bp.ns = "centerline_branches";
    bp.id = 0;
    bp.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    bp.action = visualization_msgs::msg::Marker::ADD;
    bp.scale.x = 0.25; bp.scale.y = 0.25; bp.scale.z = 0.25;
    bp.color.a = 0.9;
    bp.color.r = 1.0; bp.color.g = 0.0; bp.color.b = 0.0;
    bp.pose.orientation.w = 1.0;
    for (const auto & n : graph.branch_points) {
      geometry_msgs::msg::Point p;
      p.x = n.wx; p.y = n.wy; p.z = 0.05;
      bp.points.push_back(p);
    }
    arr.markers.push_back(make_deleteall("centerline_branches"));
    arr.markers.push_back(bp);
  }

  // ── Endpoints (green SPHERE) ───────────────────────────────
  if (!graph.endpoints.empty()) {
    visualization_msgs::msg::Marker ep;
    ep.header.frame_id = "map";
    ep.header.stamp = now;
    ep.ns = "centerline_endpoints";
    ep.id = 0;
    ep.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    ep.action = visualization_msgs::msg::Marker::ADD;
    ep.scale.x = 0.20; ep.scale.y = 0.20; ep.scale.z = 0.20;
    ep.color.a = 0.9;
    ep.color.r = 0.0; ep.color.g = 1.0; ep.color.b = 0.0;
    ep.pose.orientation.w = 1.0;
    for (const auto & n : graph.endpoints) {
      geometry_msgs::msg::Point p;
      p.x = n.wx; p.y = n.wy; p.z = 0.05;
      ep.points.push_back(p);
    }
    arr.markers.push_back(make_deleteall("centerline_endpoints"));
    arr.markers.push_back(ep);
  }

  if (!arr.markers.empty())
    marker_pub_->publish(std::move(arr));
}

}  // namespace tunnel_centerline_extractor
