// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#include "algo_core/graph/astar.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "algo_core/registry.hpp"
#include "algo_core/search_utils.hpp"

namespace algo_core
{

AstarPlanner::AstarPlanner(SearchStrategy strategy, std::string algo_name)
: strategy_(strategy), algo_name_(std::move(algo_name))
{
}

PlanResult AstarPlanner::plan(
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
  if (!grid.worldToGrid(start.x, start.y, sx, sy)) {
    result.message = "start outside costmap";
    return result;
  }
  if (!grid.worldToGrid(goal.x, goal.y, gx, gy)) {
    result.message = "goal outside costmap";
    return result;
  }

  // The start and goal poses come from AMCL and the search-viewpoint generator,
  // so either can land a few centimetres inside an inflated cell. Snapping to
  // the nearest free cell is what Nav2's own planners effectively do.
  const int snap_radius = paramInt("snap_radius", 6);
  Cell s{};
  Cell g{};
  if (!grid.nearestFree(sx, sy, snap_radius, s)) {
    result.message = "no free cell near start";
    return result;
  }
  if (!grid.nearestFree(gx, gy, snap_radius, g)) {
    result.message = "no free cell near goal";
    return result;
  }

  const Heuristic h_type = heuristicFromName(
    paramBool("allow_diagonal", true) ? "octile" : "manhattan");
  const double weight = param("weight", 2.0);
  const double cost_scale = param("cost_scale", 1.0);
  const bool allow_diagonal = paramBool("allow_diagonal", true);

  const int nx = static_cast<int>(grid.nx());
  const int total = nx * static_cast<int>(grid.ny());
  const int start_index = grid.index(s.x, s.y);
  const int goal_index = grid.index(g.x, g.y);

  std::vector<double> g_score(static_cast<std::size_t>(total), kInf);
  std::vector<int> parent(static_cast<std::size_t>(total), -1);
  std::vector<char> closed(static_cast<std::size_t>(total), 0);
  std::vector<char> opened(static_cast<std::size_t>(total), 0);

  auto priority = [&](double g_val, int x, int y) {
      const double h = heuristic(h_type, g.x - x, g.y - y);
      switch (strategy_) {
        case SearchStrategy::Dijkstra:
          return g_val;
        case SearchStrategy::GreedyBestFirst:
          return h;
        case SearchStrategy::WeightedAStar:
          return g_val + weight * h;
        case SearchStrategy::AStar:
        default:
          return g_val + h;
      }
    };

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open.emplace(priority(0.0, s.x, s.y), start_index);
  opened[static_cast<std::size_t>(start_index)] = 1;

  const int dir_count = allow_diagonal ? 8 : 4;
  bool found = false;

  while (!open.empty()) {
    const auto [f, current] = open.top();
    open.pop();
    (void)f;
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

    const double g_here = g_score[static_cast<std::size_t>(current)];
    for (int d = 0; d < dir_count; ++d) {
      const int nxp = cx + kDirX[d];
      const int nyp = cy + kDirY[d];
      if (!grid.free(nxp, nyp)) {continue;}

      // Refuse to cut a lethal corner diagonally: a footprint that cannot pass
      // between two touching obstacles must not be planned through the gap.
      if (d >= 4 && (!grid.free(cx + kDirX[d], cy) || !grid.free(cx, cy + kDirY[d]))) {
        continue;
      }

      const int neighbour = grid.index(nxp, nyp);
      if (closed[static_cast<std::size_t>(neighbour)]) {continue;}

      const double step =
        kDirCost[d] * traversalCost(grid, nxp, nyp, cost_scale);
      const double tentative = g_here + step;
      if (tentative >= g_score[static_cast<std::size_t>(neighbour)]) {continue;}

      g_score[static_cast<std::size_t>(neighbour)] = tentative;
      parent[static_cast<std::size_t>(neighbour)] = current;
      open.emplace(priority(tentative, nxp, nyp), neighbour);
      opened[static_cast<std::size_t>(neighbour)] = 1;
    }
  }

  if (!found) {
    result.message = "no path found";
  } else {
    result.success = true;
    result.cost = g_score[static_cast<std::size_t>(goal_index)];
    result.path = reconstructPath(grid, parent, goal_index);
    if (paramBool("remove_collinear", true)) {removeCollinear(result.path);}
    assignHeadings(result.path);
    result.message = "ok";
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.planning_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace algo_core

ALGO_CORE_REGISTER(algo_core::AStarPlanner, "astar")
ALGO_CORE_REGISTER(algo_core::DijkstraPlanner, "dijkstra")
ALGO_CORE_REGISTER(algo_core::GbfsPlanner, "gbfs")
ALGO_CORE_REGISTER(algo_core::WeightedAStarPlanner, "weighted_astar")
