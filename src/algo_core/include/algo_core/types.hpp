// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// ROS-free core types for the algorithm plugin library.
//
// Nothing in algo_core depends on ROS. The Nav2 adapters live in
// algo_nav2_plugins and translate between these types and ROS messages.

#ifndef ALGO_CORE__TYPES_HPP_
#define ALGO_CORE__TYPES_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace algo_core
{

/// Planar pose in world coordinates (metres, radians).
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/// A cell coordinate in the grid.
struct Cell
{
  int x{0};
  int y{0};

  bool operator==(const Cell & o) const {return x == o.x && y == o.y;}
};

/// Lightweight, ROS-free view over an occupancy/cost grid.
///
/// The adapter fills this from nav2_costmap_2d. Costs follow the ROS
/// convention: 0 = free, 255 = lethal, values in between = inflated.
class CostGrid
{
public:
  CostGrid() = default;

  CostGrid(
    unsigned int nx, unsigned int ny, double resolution,
    double origin_x, double origin_y, std::vector<uint8_t> data,
    uint8_t lethal_threshold = 253)
  : nx_(nx), ny_(ny), resolution_(resolution),
    origin_x_(origin_x), origin_y_(origin_y),
    data_(std::move(data)), lethal_threshold_(lethal_threshold)
  {
  }

  unsigned int nx() const {return nx_;}
  unsigned int ny() const {return ny_;}
  double resolution() const {return resolution_;}
  double originX() const {return origin_x_;}
  double originY() const {return origin_y_;}
  bool empty() const {return nx_ == 0 || ny_ == 0 || data_.empty();}

  bool inside(int x, int y) const
  {
    return x >= 0 && y >= 0 &&
           static_cast<unsigned int>(x) < nx_ && static_cast<unsigned int>(y) < ny_;
  }

  int index(int x, int y) const {return y * static_cast<int>(nx_) + x;}

  uint8_t cost(int x, int y) const
  {
    if (!inside(x, y)) {return 255;}
    return data_[static_cast<std::size_t>(index(x, y))];
  }

  /// True if the cell blocks the robot. Unknown space (255) counts as blocked,
  /// matching Nav2's default `track_unknown_space: false` behaviour once the
  /// adapter has collapsed unknown cells.
  bool lethal(int x, int y) const {return cost(x, y) >= lethal_threshold_;}

  bool free(int x, int y) const {return inside(x, y) && !lethal(x, y);}

  /// World -> grid. Returns false when the point falls outside the grid.
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const
  {
    if (resolution_ <= 0.0) {return false;}
    const double fx = (wx - origin_x_) / resolution_;
    const double fy = (wy - origin_y_) / resolution_;
    if (fx < 0.0 || fy < 0.0) {return false;}
    gx = static_cast<int>(std::floor(fx));
    gy = static_cast<int>(std::floor(fy));
    return inside(gx, gy);
  }

  /// Grid cell centre -> world.
  void gridToWorld(int gx, int gy, double & wx, double & wy) const
  {
    wx = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
  }

  /// Nearest free cell to (gx, gy) within `radius` cells, searching outward.
  bool nearestFree(int gx, int gy, int radius, Cell & out) const
  {
    if (free(gx, gy)) {out = Cell{gx, gy}; return true;}
    for (int r = 1; r <= radius; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {continue;}
          if (free(gx + dx, gy + dy)) {out = Cell{gx + dx, gy + dy}; return true;}
        }
      }
    }
    return false;
  }

  const std::vector<uint8_t> & data() const {return data_;}
  uint8_t lethalThreshold() const {return lethal_threshold_;}

private:
  unsigned int nx_{0};
  unsigned int ny_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<uint8_t> data_;
  uint8_t lethal_threshold_{253};
};

/// Outcome of a single planning call, with the instrumentation the benchmark
/// reports need.
struct PlanResult
{
  bool success{false};
  std::vector<Pose2D> path;                  ///< world frame, start -> goal
  std::vector<Cell> expanded;                ///< cells expanded, for visualisation
  double cost{0.0};                          ///< path cost in grid units
  std::size_t iterations{0};                 ///< nodes popped from the queue
  double planning_time_ms{0.0};              ///< wall-clock planning time
  std::string message;                       ///< human-readable status
};

/// Parameters handed down from ROS as a flat string->double map, so that
/// algo_core stays free of any parameter framework.
using ParamMap = std::unordered_map<std::string, double>;

}  // namespace algo_core

#endif  // ALGO_CORE__TYPES_HPP_
