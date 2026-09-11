// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#include "algo_core/graph/d_star_lite.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "algo_core/registry.hpp"
#include "algo_core/search_utils.hpp"

namespace algo_core
{

namespace
{
constexpr double kBig = 1e308;
}

void DStarLitePlanner::reset()
{
  initialised_ = false;
  cost_.clear();
  g_.clear();
  rhs_.clear();
  queued_.clear();
  stamp_.clear();
  next_stamp_ = 1;
  queue_ = decltype(queue_)();
  start_ = Cell{-1, -1};
  goal_ = Cell{-1, -1};
  km_ = 0.0;
}

std::vector<Cell> DStarLitePlanner::neighbours(int index) const
{
  std::vector<Cell> out;
  const int x = index % nx_;
  const int y = index / nx_;
  for (int d = 0; d < 8; ++d) {
    const int nx = x + kDirX[d];
    const int ny = y + kDirY[d];
    if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {continue;}
    if (static_cast<std::uint8_t>(cost_[static_cast<std::size_t>(ny * nx_ + nx)]) >=
      lethal_threshold_)
    {
      continue;
    }
    if (d >= 4 &&
      (cost_[static_cast<std::size_t>(y * nx_ + x + kDirX[d])] >= lethal_threshold_ ||
      cost_[static_cast<std::size_t>((y + kDirY[d]) * nx_ + x)] >= lethal_threshold_))
    {
      continue;
    }
    out.push_back(Cell{nx, ny});
  }
  return out;
}

double DStarLitePlanner::cost(int from_index, int to_index) const
{
  const int fx = from_index % nx_;
  const int fy = from_index / nx_;
  const int tx = to_index % nx_;
  const int ty = to_index / nx_;
  const double step = std::hypot(
    static_cast<double>(tx - fx), static_cast<double>(ty - fy));
  const double c = static_cast<double>(cost_[static_cast<std::size_t>(to_index)]);
  return step * (1.0 + cost_scale_ * (c / 252.0));
}

DStarLitePlanner::Key DStarLitePlanner::calculateKey(int index) const
{
  const double best = std::min(g_[static_cast<std::size_t>(index)],
      rhs_[static_cast<std::size_t>(index)]);
  const int x = index % nx_;
  const int y = index / nx_;
  Key key;
  key.k1 = best + heuristic(Heuristic::Octile, start_.x - x, start_.y - y) + km_;
  key.k2 = best;
  return key;
}

void DStarLitePlanner::insert(int index, const Key & key)
{
  queued_[static_cast<std::size_t>(index)] = key;
  const std::uint64_t stamp = next_stamp_++;
  stamp_[static_cast<std::size_t>(index)] = stamp;
  queue_.push(QueueEntry{key, index, stamp});
}

void DStarLitePlanner::remove(int index)
{
  queued_[static_cast<std::size_t>(index)] = Key{};
  stamp_[static_cast<std::size_t>(index)] = next_stamp_++;
}

bool DStarLitePlanner::topKey(Key & out)
{
  while (!queue_.empty()) {
    const QueueEntry & top = queue_.top();
    if (stamp_[static_cast<std::size_t>(top.index)] == top.stamp) {
      out = top.key;
      return true;
    }
    queue_.pop();
  }
  return false;
}

void DStarLitePlanner::updateVertex(int index)
{
  const std::size_t ui = static_cast<std::size_t>(index);
  if (index != (goal_.y * nx_ + goal_.x)) {
    double best = kBig;
    for (const Cell & nb : neighbours(index)) {
      const int ni = nb.y * nx_ + nb.x;
      if (g_[static_cast<std::size_t>(ni)] >= kBig) {continue;}
      best = std::min(best, cost(index, ni) + g_[static_cast<std::size_t>(ni)]);
    }
    rhs_[ui] = best;
  }
  if (queued_[ui] != Key{}) {remove(index);}
  if (g_[ui] != rhs_[ui]) {insert(index, calculateKey(index));}
}

void DStarLitePlanner::initialize(
  const CostGrid & grid, const Cell & start, const Cell & goal)
{
  nx_ = static_cast<int>(grid.nx());
  ny_ = static_cast<int>(grid.ny());
  resolution_ = grid.resolution();
  origin_x_ = grid.originX();
  origin_y_ = grid.originY();
  lethal_threshold_ = grid.lethalThreshold();
  cost_ = grid.data();

  const std::size_t total = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
  g_.assign(total, kBig);
  rhs_.assign(total, kBig);
  queued_.assign(total, Key{});
  stamp_.assign(total, 0);
  next_stamp_ = 1;
  queue_ = decltype(queue_)();

  start_ = start;
  goal_ = goal;
  km_ = 0.0;

  rhs_[static_cast<std::size_t>(goal.y * nx_ + goal.x)] = 0.0;
  insert(goal.y * nx_ + goal.x, calculateKey(goal.y * nx_ + goal.x));
  initialised_ = true;
}

void DStarLitePlanner::computeShortestPath()
{
  Key start_key = calculateKey(start_.y * nx_ + start_.x);
  Key top;
  while (topKey(top)) {
    // Terminate when the queue can no longer improve the start node.
    if (!(top.less(start_key) ||
      rhs_[static_cast<std::size_t>(start_.y * nx_ + start_.x)] !=
      g_[static_cast<std::size_t>(start_.y * nx_ + start_.x)]))
    {
      break;
    }

    const QueueEntry entry = queue_.top();
    queue_.pop();
    if (stamp_[static_cast<std::size_t>(entry.index)] != entry.stamp) {continue;}

    const std::size_t ui = static_cast<std::size_t>(entry.index);
    const Key current_key = calculateKey(entry.index);

    if (entry.key.less(current_key)) {
      insert(entry.index, current_key);
    } else if (g_[ui] > rhs_[ui]) {
      g_[ui] = rhs_[ui];
      remove(entry.index);
      for (const Cell & nb : neighbours(entry.index)) {
        updateVertex(nb.y * nx_ + nb.x);
      }
    } else {
      g_[ui] = kBig;
      remove(entry.index);
      updateVertex(entry.index);
      for (const Cell & nb : neighbours(entry.index)) {
        updateVertex(nb.y * nx_ + nb.x);
      }
    }
    start_key = calculateKey(start_.y * nx_ + start_.x);
  }
}

PlanResult DStarLitePlanner::plan(
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

  cost_scale_ = param("cost_scale", 1.0);

  const bool topology_changed =
    !initialised_ ||
    nx_ != static_cast<int>(grid.nx()) ||
    ny_ != static_cast<int>(grid.ny()) ||
    cost_.size() != grid.data().size() ||
    goal_.x != g.x || goal_.y != g.y;

  if (topology_changed) {
    reset();
    initialize(grid, s, g);
  } else {
    // The robot moved: carry the accumulated distance into the key offset.
    km_ += heuristic(Heuristic::Octile, start_.x - s.x, start_.y - s.y);
    start_ = s;

    // Diff the costmap and repair only around what actually changed. This is
    // the whole point of D* Lite: the diff is O(cells), the search is not.
    const std::vector<std::uint8_t> & incoming = grid.data();
    for (std::size_t i = 0; i < incoming.size(); ++i) {
      if (incoming[i] == cost_[i]) {continue;}
      cost_[i] = incoming[i];
      const int x = static_cast<int>(i) % nx_;
      const int y = static_cast<int>(i) / nx_;
      updateVertex(static_cast<int>(i));
      for (const Cell & nb : neighbours(static_cast<int>(i))) {
        updateVertex(nb.y * nx_ + nb.x);
      }
      (void)x;
      (void)y;
    }
  }

  computeShortestPath();

  const int start_index = start_.y * nx_ + start_.x;
  const int goal_index = goal_.y * nx_ + goal_.x;

  if (g_[static_cast<std::size_t>(start_index)] >= kBig) {
    result.message = "no path found";
  } else {
    // Greedy descent on g: at each step take the neighbour with the lowest
    // cost-to-goal. This is how D* Lite extracts a path after the repair.
    std::vector<Cell> cells;
    int current = start_index;
    const int guard = nx_ * ny_;
    int steps = 0;
    cells.push_back(Cell{current % nx_, current / nx_});

    while (current != goal_index && steps++ < guard) {
      int best_index = -1;
      double best_value = kBig;
      for (const Cell & nb : neighbours(current)) {
        const int ni = nb.y * nx_ + nb.x;
        if (g_[static_cast<std::size_t>(ni)] >= kBig) {continue;}
        const double value = cost(current, ni) + g_[static_cast<std::size_t>(ni)];
        if (value < best_value - 1e-9) {
          best_value = value;
          best_index = ni;
        }
      }
      if (best_index < 0) {break;}
      current = best_index;
      cells.push_back(Cell{current % nx_, current / nx_});
    }

    if (current != goal_index) {
      result.message = "path extraction failed";
    } else {
      result.success = true;
      result.cost = g_[static_cast<std::size_t>(start_index)];
      result.path.reserve(cells.size());
      for (const Cell & c : cells) {
        double wx = 0.0;
        double wy = 0.0;
        grid.gridToWorld(c.x, c.y, wx, wy);
        result.path.push_back(Pose2D{wx, wy, 0.0});
      }
      if (paramBool("remove_collinear", true)) {removeCollinear(result.path);}
      assignHeadings(result.path);
      result.message = "ok";
      for (const Cell & c : cells) {result.expanded.push_back(c);}
      result.iterations = cells.size();
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.planning_time_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

}  // namespace algo_core

ALGO_CORE_REGISTER(algo_core::DStarLitePlanner, "d_star_lite")
