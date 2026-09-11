// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// Shared helpers for grid search: heuristics, line-of-sight, path rebuilding.

#ifndef ALGO_CORE__SEARCH_UTILS_HPP_
#define ALGO_CORE__SEARCH_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "algo_core/types.hpp"

namespace algo_core
{

inline constexpr double kInf = std::numeric_limits<double>::infinity();

/// The eight connected neighbour offsets, ordered so that the first four are
/// the axis-aligned moves.
inline constexpr int kDirX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
inline constexpr int kDirY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

/// Movement cost multiplier per direction, relative to one cell step.
inline constexpr double kDirCost[8] = {
  1.0, 1.0, 1.0, 1.0,
  std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

enum class Heuristic
{
  Euclidean,
  Manhattan,
  Octile,
  Chebyshev,
  Zero,        ///< turns A* into Dijkstra
};

inline double heuristic(Heuristic type, int dx, int dy)
{
  const double ax = std::abs(static_cast<double>(dx));
  const double ay = std::abs(static_cast<double>(dy));
  switch (type) {
    case Heuristic::Euclidean:
      return std::sqrt(ax * ax + ay * ay);
    case Heuristic::Manhattan:
      return ax + ay;
    case Heuristic::Octile: {
      const double mn = std::min(ax, ay);
      return (ax + ay) + (std::sqrt(2.0) - 2.0) * mn;
    }
    case Heuristic::Chebyshev:
      return std::max(ax, ay);
    case Heuristic::Zero:
    default:
      return 0.0;
  }
}

inline Heuristic heuristicFromName(const std::string & name)
{
  if (name == "euclidean") {return Heuristic::Euclidean;}
  if (name == "manhattan") {return Heuristic::Manhattan;}
  if (name == "chebyshev") {return Heuristic::Chebyshev;}
  if (name == "zero" || name == "dijkstra") {return Heuristic::Zero;}
  return Heuristic::Octile;
}

/// True when no lethal cell lies on the segment between two cell centres.
///
/// Uses a dense sampling test rather than Bresenham: sampling is slightly more
/// permissive at cell corners, which is what a physical robot footprint wants,
/// and it keeps the check symmetric between (a,b) and (b,a).
inline bool lineOfSight(const CostGrid & grid, int x0, int y0, int x1, int y1)
{
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int steps = std::max(dx, dy);
  if (steps == 0) {return grid.free(x0, y0);}

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const int x = static_cast<int>(std::lround(x0 + t * (x1 - x0)));
    const int y = static_cast<int>(std::lround(y0 + t * (y1 - y0)));
    if (!grid.free(x, y)) {return false;}
  }
  return true;
}

/// Cost of entering a cell: unit step times the cell's traversal penalty.
/// `cost_scale` converts the 0..252 costmap range into a multiplier >= 1.
inline double traversalCost(const CostGrid & grid, int x, int y, double cost_scale)
{
  if (cost_scale <= 0.0) {return 1.0;}
  const double c = static_cast<double>(grid.cost(x, y));
  return 1.0 + cost_scale * (c / 252.0);
}

/// Walk a came-from map backwards and emit world-frame poses.
inline std::vector<Pose2D> reconstructPath(
  const CostGrid & grid, const std::vector<int> & parent, int goal_index)
{
  std::vector<Pose2D> path;
  int current = goal_index;
  while (current >= 0) {
    const int x = current % static_cast<int>(grid.nx());
    const int y = current / static_cast<int>(grid.nx());
    double wx = 0.0;
    double wy = 0.0;
    grid.gridToWorld(x, y, wx, wy);
    path.push_back(Pose2D{wx, wy, 0.0});
    current = parent[static_cast<std::size_t>(current)];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

/// Fill in heading for each pose from the direction of travel. The final pose
/// keeps the heading of the last segment.
inline void assignHeadings(std::vector<Pose2D> & path)
{
  if (path.size() < 2) {return;}
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].theta = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  path.back().theta = path[path.size() - 2].theta;
}

/// Remove collinear intermediate points. Purely cosmetic for path messages, and
/// it keeps the reported path length honest (no double counting of stair steps).
inline void removeCollinear(std::vector<Pose2D> & path)
{
  if (path.size() < 3) {return;}
  std::vector<Pose2D> out;
  out.push_back(path.front());
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double cross =
      (path[i].x - out.back().x) * (path[i + 1].y - out.back().y) -
      (path[i].y - out.back().y) * (path[i + 1].x - out.back().x);
    if (std::abs(cross) > 1e-9) {out.push_back(path[i]);}
  }
  out.push_back(path.back());
  path.swap(out);
}

}  // namespace algo_core

#endif  // ALGO_CORE__SEARCH_UTILS_HPP_
