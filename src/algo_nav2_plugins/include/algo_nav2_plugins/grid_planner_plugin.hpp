// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// Nav2 adapter: exposes every registered algo_core algorithm through the
// nav2_core::GlobalPlanner interface.
//
// One plugin class serves all algorithms. Which algorithm runs is decided by
// the `algorithm` parameter, resolved through the algo_core registry at
// configure() time. That is what makes switching algorithm a configuration
// change: the plugin class, the behaviour tree and the launch file all stay
// exactly as they are.

#ifndef ALGO_NAV2_PLUGINS__GRID_PLANNER_PLUGIN_HPP_
#define ALGO_NAV2_PLUGINS__GRID_PLANNER_PLUGIN_HPP_

#include <functional>
#include <memory>
#include <string>

#include "algo_core/grid_planner.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace algo_nav2_plugins
{

class GridPlannerPlugin : public nav2_core::GlobalPlanner
{
public:
  GridPlannerPlugin() = default;
  ~GridPlannerPlugin() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  /// Name of the algorithm currently bound to this plugin instance.
  const std::string & algorithm() const {return algorithm_name_;}

private:
  /// Snapshot the live costmap into the ROS-free representation.
  algo_core::CostGrid toCostGrid() const;

  /// Publish the expanded set for debugging, when enabled.
  void publishExpanded(const algo_core::PlanResult & result);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("algo_nav2_plugins")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{nullptr};

  std::string plugin_name_;
  std::string algorithm_name_;

  algo_core::GridPlanner::Ptr planner_;

  // Mirror of the parameters handed to algo_core, kept so that configure()
  // stays the only place that knows about ROS parameters.
  algo_core::ParamMap params_;

  uint8_t lethal_threshold_{253};

  bool publish_expanded_{false};
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    expanded_pub_;
};

}  // namespace algo_nav2_plugins

#endif  // ALGO_NAV2_PLUGINS__GRID_PLANNER_PLUGIN_HPP_
