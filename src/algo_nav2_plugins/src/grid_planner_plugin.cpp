// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#include "algo_nav2_plugins/grid_planner_plugin.hpp"

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "algo_core/registry.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace algo_nav2_plugins
{

namespace
{
/// Parameter keys forwarded to algo_core. Kept as data rather than as a list of
/// hand-written get_parameter calls so that adding an algorithm parameter does
/// not require touching the adapter.
constexpr const char * kForwardedParams[] = {
  "cost_scale",
  "snap_radius",
  "weight",
  "allow_diagonal",
  "remove_collinear",
  "smooth",
};

std::string join(const std::vector<std::string> & items, const char * sep)
{
  std::ostringstream out;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {out << sep;}
    out << items[i];
  }
  return out.str();
}
}  // namespace

void GridPlannerPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("GridPlannerPlugin: parent node is gone");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".algorithm", rclcpp::ParameterValue(std::string("astar")));
  algorithm_name_ = node->get_parameter(name + ".algorithm").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lethal_threshold", rclcpp::ParameterValue(253));
  lethal_threshold_ = static_cast<uint8_t>(
    node->get_parameter(name + ".lethal_threshold").as_int());

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".publish_expanded", rclcpp::ParameterValue(false));
  publish_expanded_ = node->get_parameter(name + ".publish_expanded").as_bool();

  // Forward only the algorithm parameters that the YAML actually specified.
  //
  // Declaring every known key with a blanket default of 0.0 silently overrides
  // the algorithms' own defaults — `allow_diagonal` would arrive as false and
  // `snap_radius` as 0 — so the parameter overrides are inspected instead, and
  // anything absent is left for the algorithm to decide.
  params_.clear();
  const auto overrides =
    node->get_node_parameters_interface()->get_parameter_overrides();
  for (const char * key : kForwardedParams) {
    const std::string full = name + "." + key;
    if (overrides.find(full) == overrides.end()) {continue;}

    nav2_util::declare_parameter_if_not_declared(
      node, full, rclcpp::ParameterValue(0.0));
    const auto & parameter = node->get_parameter(full);
    switch (parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        params_[key] = parameter.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        params_[key] = static_cast<double>(parameter.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        params_[key] = parameter.as_bool() ? 1.0 : 0.0;
        break;
      default:
        break;
    }
  }

  planner_ = algo_core::Registry::instance().create(algorithm_name_);
  if (!planner_) {
    const std::vector<std::string> available =
      algo_core::Registry::instance().availableNames();
    RCLCPP_ERROR(
      logger_, "[%s] unknown algorithm '%s'. Available: %s",
      plugin_name_.c_str(), algorithm_name_.c_str(),
      join(available, ", ").c_str());
    throw std::runtime_error(
            "GridPlannerPlugin: unknown algorithm '" + algorithm_name_ + "'");
  }
  planner_->configure(params_);

  if (publish_expanded_) {
    expanded_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      plugin_name_ + "/expanded", 1);
  }

  RCLCPP_INFO(
    logger_, "[%s] algorithm '%s' ready (%zu algorithms registered)",
    plugin_name_.c_str(), algorithm_name_.c_str(),
    algo_core::Registry::instance().availableNames().size());
}

void GridPlannerPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "[%s] cleaning up", plugin_name_.c_str());
  expanded_pub_.reset();
  planner_.reset();
}

void GridPlannerPlugin::activate()
{
  RCLCPP_INFO(logger_, "[%s] activating", plugin_name_.c_str());
  if (expanded_pub_) {expanded_pub_->on_activate();}
}

void GridPlannerPlugin::deactivate()
{
  RCLCPP_INFO(logger_, "[%s] deactivating", plugin_name_.c_str());
  if (expanded_pub_) {expanded_pub_->on_deactivate();}
}

algo_core::CostGrid GridPlannerPlugin::toCostGrid() const
{
  if (!costmap_ros_) {return algo_core::CostGrid();}

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  {
    // Copy out under the costmap lock: createPlan runs on the planner thread
    // while the costmap thread may be midway through an update.
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros_->getCostmap()->getMutex()));
    costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(*costmap_ros_->getCostmap());
  }

  const unsigned int nx = costmap->getSizeInCellsX();
  const unsigned int ny = costmap->getSizeInCellsY();
  const unsigned char * raw = costmap->getCharMap();

  std::vector<uint8_t> data(raw, raw + static_cast<std::size_t>(nx) * ny);
  return algo_core::CostGrid(
    nx, ny, costmap->getResolution(),
    costmap->getOriginX(), costmap->getOriginY(),
    std::move(data), lethal_threshold_);
}

void GridPlannerPlugin::publishExpanded(const algo_core::PlanResult & result)
{
  if (!expanded_pub_ || result.expanded.empty()) {return;}

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  marker.header.stamp = clock_->now();
  marker.ns = plugin_name_ + "_expanded_" + algorithm_name_;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = costmap_ros_->getCostmap()->getResolution();
  marker.scale.y = costmap_ros_->getCostmap()->getResolution();
  marker.color.r = 1.0f;
  marker.color.g = 0.4f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  const auto & costmap = *costmap_ros_->getCostmap();
  for (const algo_core::Cell & cell : result.expanded) {
    geometry_msgs::msg::Point point;
    point.x = costmap.getOriginX() + (cell.x + 0.5) * costmap.getResolution();
    point.y = costmap.getOriginY() + (cell.y + 0.5) * costmap.getResolution();
    point.z = 0.05;
    marker.points.push_back(point);
  }
  array.markers.push_back(marker);
  expanded_pub_->publish(array);
}

nav_msgs::msg::Path GridPlannerPlugin::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!planner_) {
    RCLCPP_ERROR(logger_, "[%s] planner not configured", plugin_name_.c_str());
    return path;
  }

  if (cancel_checker && cancel_checker()) {
    RCLCPP_DEBUG(logger_, "[%s] plan cancelled before it started", plugin_name_.c_str());
    return path;
  }

  const algo_core::CostGrid grid = toCostGrid();
  if (grid.empty()) {
    RCLCPP_ERROR(logger_, "[%s] costmap is empty", plugin_name_.c_str());
    return path;
  }

  // Poses arrive in the global frame and algo_core's CostGrid works in world
  // coordinates too (CostGrid::worldToGrid applies the costmap origin itself),
  // so the poses are handed over unchanged. Subtracting the origin here as well
  // would apply it twice and land the search in the wrong cell.
  const algo_core::Pose2D from{
    start.pose.position.x, start.pose.position.y, 0.0};
  const algo_core::Pose2D to{
    goal.pose.position.x, goal.pose.position.y, 0.0};

  const algo_core::PlanResult result = planner_->plan(grid, from, to);

  if (!result.success) {
    RCLCPP_WARN(
      logger_, "[%s/%s] planning failed: %s (%.2f ms)",
      plugin_name_.c_str(), algorithm_name_.c_str(),
      result.message.c_str(), result.planning_time_ms);
    return path;
  }

  path.poses.reserve(result.path.size());
  for (const algo_core::Pose2D & pose : result.path) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = path.header;
    // Already world coordinates: CostGrid::gridToWorld added the origin.
    stamped.pose.position.x = pose.x;
    stamped.pose.position.y = pose.y;
    stamped.pose.position.z = 0.0;
    stamped.pose.orientation.z = std::sin(pose.theta * 0.5);
    stamped.pose.orientation.w = std::cos(pose.theta * 0.5);
    path.poses.push_back(stamped);
  }

  publishExpanded(result);

  RCLCPP_DEBUG(
    logger_, "[%s/%s] %zu poses, %.2f ms, %zu expanded",
    plugin_name_.c_str(), algorithm_name_.c_str(), path.poses.size(),
    result.planning_time_ms, result.expanded.size());

  return path;
}

}  // namespace algo_nav2_plugins

PLUGINLIB_EXPORT_CLASS(algo_nav2_plugins::GridPlannerPlugin, nav2_core::GlobalPlanner)
