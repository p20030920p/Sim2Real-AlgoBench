// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#ifndef ALGO_CORE__GRAPH__THETA_STAR_HPP_
#define ALGO_CORE__GRAPH__THETA_STAR_HPP_

#include <string>

#include "algo_core/grid_planner.hpp"

namespace algo_core
{

/// Theta*: A* with any-angle path relaxation.
///
/// When a cell can see its grandparent, the parent pointer is rewired to the
/// grandparent, so the resulting path is not restricted to the 8 grid headings.
/// Included here alongside Nav2's own Theta* so that the two can be compared
/// under one cost model and one set of tie-breaking rules.
class ThetaStarPlanner final : public GridPlanner
{
public:
  std::string name() const override {return "theta_star";}

  PlanResult plan(const CostGrid & grid, const Pose2D & start, const Pose2D & goal) override;
};

}  // namespace algo_core

#endif  // ALGO_CORE__GRAPH__THETA_STAR_HPP_
