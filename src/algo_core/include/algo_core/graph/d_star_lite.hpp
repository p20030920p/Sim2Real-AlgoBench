// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#ifndef ALGO_CORE__GRAPH__D_STAR_LITE_HPP_
#define ALGO_CORE__GRAPH__D_STAR_LITE_HPP_

#include <cstdint>
#include <queue>
#include <string>
#include <vector>

#include "algo_core/grid_planner.hpp"

namespace algo_core
{

/// D* Lite (Koenig & Likhachev).
///
/// Searches backwards from the goal and repairs its solution incrementally when
/// edge costs change. The reason it earns its place in this library is that a
/// Nav2 costmap changes on every cycle — moving obstacles in the stress world
/// are exactly the situation D* Lite was designed for.
///
/// The incremental behaviour only happens if the same instance is reused across
/// calls, so the planner keeps its g/rhs arrays, its queue and the previous
/// costmap between invocations, and diffs the costmap to find changed cells.
/// Rebuilding the instance every cycle throws the entire advantage away.
///
/// The grid is treated as undirected (successors and predecessors are both the
/// 8-neighbourhood), which is what a holonomic robot on a grid actually has.
class DStarLitePlanner final : public GridPlanner
{
public:
  std::string name() const override {return "d_star_lite";}

  PlanResult plan(const CostGrid & grid, const Pose2D & start, const Pose2D & goal) override;

  /// Drop all cached state; the next plan() call starts from scratch.
  void reset();

private:
  struct Key
  {
    double k1{kInfLiteral()};
    double k2{kInfLiteral()};

    static constexpr double kInfLiteral() {return 1e308;}

    bool operator==(const Key & o) const {return k1 == o.k1 && k2 == o.k2;}
    bool operator!=(const Key & o) const {return !(*this == o);}
    bool less(const Key & o) const
    {
      if (k1 != o.k1) {return k1 < o.k1;}
      return k2 < o.k2;
    }
  };

  struct QueueEntry
  {
    Key key;
    int index{0};
    std::uint64_t stamp{0};
  };

  struct QueueCompare
  {
    bool operator()(const QueueEntry & a, const QueueEntry & b) const
    {
      return b.key.less(a.key);   // min-heap on (k1, k2)
    }
  };

  void initialize(const CostGrid & grid, const Cell & start, const Cell & goal);
  void computeShortestPath();
  void updateVertex(int index);
  Key calculateKey(int index) const;
  double cost(int from_index, int to_index) const;
  void insert(int index, const Key & key);
  void remove(int index);
  bool topKey(Key & out);
  std::vector<Cell> neighbours(int index) const;

  // --- persistent incremental state ---
  bool initialised_{false};
  int nx_{0};
  int ny_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double cost_scale_{1.0};
  std::uint8_t lethal_threshold_{253};
  std::vector<std::uint8_t> cost_;     ///< costmap snapshot from the last call
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<Key> queued_;
  std::vector<std::uint64_t> stamp_;
  std::uint64_t next_stamp_{1};
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue_;
  Cell start_{-1, -1};
  Cell goal_{-1, -1};
  double km_{0.0};
};

}  // namespace algo_core

#endif  // ALGO_CORE__GRAPH__D_STAR_LITE_HPP_
