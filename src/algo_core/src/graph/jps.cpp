// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#include "algo_core/graph/jps.hpp"

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

bool JpsPlanner::jump(
  const CostGrid & grid, int x, int y, int dx, int dy,
  int goal_x, int goal_y, int & jx, int & jy)
{
  int nx = x + dx;
  int ny = y + dy;

  while (grid.free(nx, ny)) {
    if (nx == goal_x && ny == goal_y) {
      jx = nx;
      jy = ny;
      return true;
    }

    if (dx != 0 && dy != 0) {
      // Diagonal step: a jump point exists when either perpendicular ray finds
      // one, or when stepping here created a forced neighbour.
      int tx = 0;
      int ty = 0;
      if (jump(grid, nx, ny, dx, 0, goal_x, goal_y, tx, ty)) {
        jx = nx;
        jy = ny;
        return true;
      }
      if (jump(grid, nx, ny, 0, dy, goal_x, goal_y, tx, ty)) {
        jx = nx;
        jy = ny;
        return true;
      }
      const bool blocked_a = !grid.free(nx - dx, ny);
      const bool blocked_b = !grid.free(nx, ny - dy);
      if ((blocked_a && grid.free(nx - dx, ny + dy)) ||
        (blocked_b && grid.free(nx + dx, ny - dy)))
      {
        jx = nx;
        jy = ny;
        return true;
      }
    } else if (dx != 0) {
      // Horizontal ray.
      if ((!grid.free(nx, ny + 1) && grid.free(nx + dx, ny + 1)) ||
        (!grid.free(nx, ny - 1) && grid.free(nx + dx, ny - 1)))
      {
        jx = nx;
        jy = ny;
        return true;
      }
    } else {
      // Vertical ray.
      if ((!grid.free(nx + 1, ny) && grid.free(nx + 1, ny + dy)) ||
        (!grid.free(nx - 1, ny) && grid.free(nx - 1, ny + dy)))
      {
        jx = nx;
        jy = ny;
        return true;
      }
    }

    nx += dx;
    ny += dy;
  }
  return false;
}

std::vector<Pose2D> JpsPlanner::smooth(
  const CostGrid & grid, const std::vector<Pose2D> & raw)
{
  if (raw.size() < 3) {return raw;}

  std::vector<Cell> cells;
  cells.reserve(raw.size());
  for (const auto & p : raw) {
    int gx = 0;
    int gy = 0;
    if (grid.worldToGrid(p.x, p.y, gx, gy)) {cells.push_back(Cell{gx, gy});}
  }
  if (cells.size() < 3) {return raw;}

  std::vector<Pose2D> out;
  std::size_t anchor = 0;
  out.push_back(raw[anchor]);
  while (anchor + 1 < cells.size()) {
    std::size_t best = anchor + 1;
    for (std::size_t probe = cells.size(); probe > anchor + 1; --probe) {
      const std::size_t candidate = probe - 1;
      if (lineOfSight(
          grid, cells[anchor].x, cells[anchor].y,
          cells[candidate].x, cells[candidate].y))
      {
        best = candidate;
        break;
      }
    }
    out.push_back(raw[best]);
    anchor = best;
  }
  return out;
}

PlanResult JpsPlanner::plan(
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

  const int nx = static_cast<int>(grid.nx());
  const int total = nx * static_cast<int>(grid.ny());
  const int goal_index = grid.index(g.x, g.y);

  std::vector<double> g_score(static_cast<std::size_t>(total), kInf);
  std::vector<int> parent(static_cast<std::size_t>(total), -1);
  std::vector<char> closed(static_cast<std::size_t>(total), 0);

  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

  const int start_index = grid.index(s.x, s.y);
  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open.emplace(
    heuristic(Heuristic::Octile, g.x - s.x, g.y - s.y), start_index);

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

    // Direction of travel into this node, used to prune neighbours. Without a
    // parent we are at the start and every direction is allowed.
    std::vector<std::pair<int, int>> dirs;
    if (parent[static_cast<std::size_t>(current)] < 0) {
      for (int d = 0; d < 8; ++d) {dirs.emplace_back(kDirX[d], kDirY[d]);}
    } else {
      const int px = parent[static_cast<std::size_t>(current)] % nx;
      const int py = parent[static_cast<std::size_t>(current)] / nx;
      const int dx = (cx > px) ? 1 : ((cx < px) ? -1 : 0);
      const int dy = (cy > py) ? 1 : ((cy < py) ? -1 : 0);

      if (dx != 0 && dy != 0) {
        dirs.emplace_back(dx, dy);
        dirs.emplace_back(dx, 0);
        dirs.emplace_back(0, dy);
        if (grid.free(cx - dx, cy + dy)) {dirs.emplace_back(-dx, dy);}
        if (grid.free(cx + dx, cy - dy)) {dirs.emplace_back(dx, -dy);}
      } else if (dx != 0) {
        dirs.emplace_back(dx, 0);
        if (grid.free(cx, cy + 1)) {dirs.emplace_back(dx, 1);}
        if (grid.free(cx, cy - 1)) {dirs.emplace_back(dx, -1);}
      } else {
        dirs.emplace_back(0, dy);
        if (grid.free(cx + 1, cy)) {dirs.emplace_back(1, dy);}
        if (grid.free(cx - 1, cy)) {dirs.emplace_back(-1, dy);}
      }
    }

    const double g_here = g_score[static_cast<std::size_t>(current)];

    for (const auto & [dx, dy] : dirs) {
      if (dx == 0 && dy == 0) {continue;}
      // A diagonal move requires both orthogonal cells to be clear.
      if (dx != 0 && dy != 0 &&
        (!grid.free(cx + dx, cy) || !grid.free(cx, cy + dy)))
      {
        continue;
      }

      int jx = 0;
      int jy = 0;
      if (!jump(grid, cx, cy, dx, dy, g.x, g.y, jx, jy)) {continue;}

      const int jindex = grid.index(jx, jy);
      if (closed[static_cast<std::size_t>(jindex)]) {continue;}

      const double step = std::hypot(
        static_cast<double>(jx - cx), static_cast<double>(jy - cy));
      const double tentative = g_here + step;
      if (tentative >= g_score[static_cast<std::size_t>(jindex)]) {continue;}

      g_score[static_cast<std::size_t>(jindex)] = tentative;
      parent[static_cast<std::size_t>(jindex)] = current;
      open.emplace(
        tentative + heuristic(Heuristic::Octile, g.x - jx, g.y - jy), jindex);
    }
  }

  if (!found) {
    result.message = "no path found";
  } else {
    result.success = true;
    result.cost = g_score[static_cast<std::size_t>(goal_index)];
    result.path = reconstructPath(grid, parent, goal_index);
    if (paramBool("smooth", true)) {result.path = smooth(grid, result.path);}
    assignHeadings(result.path);
    result.message = "ok";
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.planning_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace algo_core

ALGO_CORE_REGISTER(algo_core::JpsPlanner, "jps")
