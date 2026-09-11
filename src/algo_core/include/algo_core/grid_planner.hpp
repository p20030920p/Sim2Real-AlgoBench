// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// Abstract interface every algorithm in this library implements.
//
// The point of this interface is that algorithms know nothing about ROS, Nav2
// or costmaps. Anything that wants to run them — a Nav2 plugin, a standalone
// node, a unit test, an offline sweep — talks to this class only.

#ifndef ALGO_CORE__GRID_PLANNER_HPP_
#define ALGO_CORE__GRID_PLANNER_HPP_

#include <memory>
#include <string>

#include "algo_core/types.hpp"

namespace algo_core
{

class GridPlanner
{
public:
  using Ptr = std::unique_ptr<GridPlanner>;

  virtual ~GridPlanner() = default;

  /// Stable identifier, matching the key used in the registry and in
  /// algo_registry.yaml.
  virtual std::string name() const = 0;

  /// Apply parameters. Called once after construction. Implementations should
  /// ignore keys they do not know, so one YAML block can carry the union of all
  /// algorithms' parameters.
  virtual void configure(const ParamMap & params) {params_ = params;}

  /// Plan from start to goal. Must be callable repeatedly on the same instance;
  /// algorithms that carry state between calls (D* Lite) may exploit that.
  virtual PlanResult plan(
    const CostGrid & grid, const Pose2D & start, const Pose2D & goal) = 0;

protected:
  /// Parameter lookup with default.
  double param(const std::string & key, double fallback) const
  {
    const auto it = params_.find(key);
    return it == params_.end() ? fallback : it->second;
  }

  /// Integer parameter lookup with default.
  int paramInt(const std::string & key, int fallback) const
  {
    return static_cast<int>(param(key, static_cast<double>(fallback)));
  }

  /// Boolean parameter lookup with default.
  bool paramBool(const std::string & key, bool fallback) const
  {
    return param(key, fallback ? 1.0 : 0.0) >= 0.5;
  }

  ParamMap params_;
};

}  // namespace algo_core

#endif  // ALGO_CORE__GRID_PLANNER_HPP_
