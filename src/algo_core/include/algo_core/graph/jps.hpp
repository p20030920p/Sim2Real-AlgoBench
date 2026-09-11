// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#ifndef ALGO_CORE__GRAPH__JPS_HPP_
#define ALGO_CORE__GRAPH__JPS_HPP_

#include <string>
#include <vector>

#include "algo_core/grid_planner.hpp"

namespace algo_core
{

/// Jump Point Search.
///
/// ASSUMPTION: JPS is defined for uniform-cost grids. This implementation
/// prunes against the lethal/free classification and therefore minimises path
/// *length*, ignoring the traversal penalties encoded in inflated cells. On a
/// Nav2 costmap with a large inflation radius that makes it a different
/// objective from A*, not a faster A* — which is exactly why it is worth
/// running both. Set `smooth:=true` to apply a line-of-sight string pull to the
/// pruned path.
class JpsPlanner final : public GridPlanner
{
public:
  std::string name() const override {return "jps";}

  PlanResult plan(const CostGrid & grid, const Pose2D & start, const Pose2D & goal) override;

private:
  /// Walk from (x,y) along (dx,dy) and report the first jump point.
  static bool jump(
    const CostGrid & grid, int x, int y, int dx, int dy,
    int goal_x, int goal_y, int & jx, int & jy);

  /// Apply line-of-sight string pulling to remove unnecessary corners.
  static std::vector<Pose2D> smooth(const CostGrid & grid, const std::vector<Pose2D> & raw);
};

}  // namespace algo_core

#endif  // ALGO_CORE__GRAPH__JPS_HPP_
