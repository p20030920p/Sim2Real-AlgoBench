// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT

#ifndef ALGO_CORE__GRAPH__ASTAR_HPP_
#define ALGO_CORE__GRAPH__ASTAR_HPP_

#include <string>

#include "algo_core/grid_planner.hpp"

namespace algo_core
{

/// Which priority function the shared grid-search skeleton uses.
enum class SearchStrategy
{
  AStar,            ///< f = g + h
  Dijkstra,         ///< f = g
  GreedyBestFirst,  ///< f = h
  WeightedAStar,    ///< f = g + weight * h
};

/// One implementation, four algorithms: the only difference between A*,
/// Dijkstra, GBFS and weighted A* is the priority function, so keeping them in
/// one place makes the comparison honest — identical data structures, identical
/// tie-breaking, identical cost model.
class AstarPlanner : public GridPlanner
{
public:
  AstarPlanner(SearchStrategy strategy, std::string algo_name);

  std::string name() const override {return algo_name_;}

  PlanResult plan(const CostGrid & grid, const Pose2D & start, const Pose2D & goal) override;

private:
  SearchStrategy strategy_;
  std::string algo_name_;
};

class AStarPlanner final : public AstarPlanner
{
public:
  AStarPlanner() : AstarPlanner(SearchStrategy::AStar, "astar") {}
};

class DijkstraPlanner final : public AstarPlanner
{
public:
  DijkstraPlanner() : AstarPlanner(SearchStrategy::Dijkstra, "dijkstra") {}
};

class GbfsPlanner final : public AstarPlanner
{
public:
  GbfsPlanner() : AstarPlanner(SearchStrategy::GreedyBestFirst, "gbfs") {}
};

class WeightedAStarPlanner final : public AstarPlanner
{
public:
  WeightedAStarPlanner() : AstarPlanner(SearchStrategy::WeightedAStar, "weighted_astar") {}
};

}  // namespace algo_core

#endif  // ALGO_CORE__GRAPH__ASTAR_HPP_
