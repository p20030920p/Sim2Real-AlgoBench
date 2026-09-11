// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#include "algo_core/graph/theta_star.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "algo_core/registry.hpp"
#include "algo_core/search_utils.hpp"

namespace algo_core
{

PlanResult ThetaStarPlanner::plan(
  const CostGrid & grid, const Pose2D & start, const Pose2D & goal)
{
  const auto t0 = std::chrono::steady_clock::now();
  PlanResult result;

  if (grid.empty()) {
    result.message = "empty costmap";
    return result;
  }

  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!grid.worldToGrid(start.x, start.y, sx, sy) ||
    !grid.worldToGrid(goal.x, goal.y, gx, gy))
  {
    result.message = "start or goal outside costmap";
    return result;
  }

  const int snap_radius = paramInt("snap_radius", 6);
  Cell s{};
  Cell g{};
  if (!grid.nearestFree(sx, sy, snap_radius, s) ||
    !grid.nearestFree(gx, gy, snap_radius, g))
  {
    result.message = "no free cell near start or goal";
    return result;
  }

  const double cost_scale = param("cost_scale", 1.0);
  const int nx = static_cast<int>(grid.nx());
  const int total = nx * static_cast<int>(grid.ny());
  const int start_index = grid.index(s.x, s.y);
  const int goal_index = grid.index(g.x, g.y);

  std::vector<double> g_score(static_cast<std::size_t>(total), kInf);
  std::vector<int> parent(static_cast<std::size_t>(total), -1);
  std::vector<char> closed(static_cast<std::size_t>(total), 0);

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open.emplace(heuristic(Heuristic::Octile, g.x - s.x, g.y - s.y), start_index);

  bool found = false;

  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[static_cast<std::size_t>(current)]) {continue;}
    closed[static_cast<std::size_t>(current)] = 1;
    ++result.iterations;

    const int cx = current % nx;
    const int cy = current / nx;
    result.expanded.push_back(Cell{cx, cy});

    if (current == goal_index) {
      found = true;
      break;
    }

    for (int d = 0; d < 8; ++d) {
      const int nxp = cx + kDirX[d];
      const int nyp = cy + kDirY[d];
      if (!grid.free(nxp, nyp)) {continue;}
      if (d >= 4 && (!grid.free(cx + kDirX[d], cy) || !grid.free(cx, cy + kDirY[d]))) {
        continue;
      }

      const int neighbour = grid.index(nxp, nyp);
      if (closed[static_cast<std::size_t>(neighbour)]) {continue;}

      const int par = parent[static_cast<std::size_t>(current)];
      double tentative = kInf;
      int new_parent = current;

      // Path 2: shortcut straight from the grandparent when it can see the
      // neighbour. This is the step that makes the path any-angle.
      if (par >= 0) {
        const int px = par % nx;
        const int py = par / nx;
        if (lineOfSight(grid, px, py, nxp, nyp)) {
          const double dist = std::hypot(
            static_cast<double>(nxp - px), static_cast<double>(nyp - py));
          tentative = g_score[static_cast<std::size_t>(par)] +
            dist * traversalCost(grid, nxp, nyp, cost_scale);
          new_parent = par;
        }
      }

      // Path 1: ordinary grid move from the current cell.
      if (!std::isfinite(tentative)) {
        tentative = g_score[static_cast<std::size_t>(current)] +
          kDirCost[d] * traversalCost(grid, nxp, nyp, cost_scale);
        new_parent = current;
      }

      if (tentative >= g_score[static_cast<std::size_t>(neighbour)]) {continue;}

      g_score[static_cast<std::size_t>(neighbour)] = tentative;
      parent[static_cast<std::size_t>(neighbour)] = new_parent;
      open.emplace(
        tentative + heuristic(Heuristic::Octile, g.x - nxp, g.y - nyp), neighbour);
    }
  }

  if (!found) {
    result.message = "no path found";
  } else {
    result.success = true;
    result.cost = g_score[static_cast<std::size_t>(goal_index)];
    result.path = reconstructPath(grid, parent, goal_index);
    assignHeadings(result.path);
    result.message = "ok";
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.planning_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace algo_core

ALGO_CORE_REGISTER(algo_core::ThetaStarPlanner, "theta_star")
