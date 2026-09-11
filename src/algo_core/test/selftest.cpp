// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// ROS-free self test for every registered algorithm.
//
// Checks the invariants that matter before an algorithm is allowed near a real
// robot: a path is found, it connects the requested endpoints, and no pose on
// it lies in a lethal cell. Also exercises D* Lite's incremental path by
// mutating the costmap between two calls on the same instance.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "algo_core/graph/d_star_lite.hpp"
#include "algo_core/registry.hpp"
#include "algo_core/search_utils.hpp"
#include "algo_core/types.hpp"

using algo_core::Cell;
using algo_core::CostGrid;
using algo_core::ParamMap;
using algo_core::Pose2D;
using algo_core::Registry;

namespace
{

int g_failures = 0;

void check(bool condition, const std::string & what)
{
  if (!condition) {
    std::printf("  FAIL  %s\n", what.c_str());
    ++g_failures;
  }
}

/// A 200x200 map at 5 cm: two rooms joined by a doorway, plus a pillar to force
/// a non-trivial route.
CostGrid makeMap()
{
  const unsigned int nx = 200;
  const unsigned int ny = 200;
  const double res = 0.05;
  std::vector<uint8_t> data(static_cast<std::size_t>(nx) * ny, 0);
  auto at = [&](unsigned int x, unsigned int y) -> uint8_t & {
      return data[static_cast<std::size_t>(y) * nx + x];
    };

  // Outer walls.
  for (unsigned int x = 0; x < nx; ++x) {at(x, 0) = 254; at(x, ny - 1) = 254;}
  for (unsigned int y = 0; y < ny; ++y) {at(0, y) = 254; at(nx - 1, y) = 254;}

  // Dividing wall at x = 100 with a doorway at y in [90, 110].
  for (unsigned int y = 1; y < ny - 1; ++y) {
    if (y >= 90 && y <= 110) {continue;}
    at(100, y) = 254;
  }

  // A pillar in the right room, to make straight lines impossible.
  for (unsigned int x = 140; x < 165; ++x) {
    for (unsigned int y = 60; y < 85; ++y) {at(x, y) = 254;}
  }
  return CostGrid(nx, ny, res, -5.0, -5.0, std::move(data), 253);
}

/// True when every pose of the path lies in a non-lethal cell.
bool pathIsFree(const CostGrid & grid, const std::vector<Pose2D> & path, std::string & why)
{
  for (const Pose2D & p : path) {
    int gx = 0;
    int gy = 0;
    if (!grid.worldToGrid(p.x, p.y, gx, gy)) {
      why = "pose outside map";
      return false;
    }
    if (!grid.free(gx, gy)) {
      why = "pose in lethal cell";
      return false;
    }
  }
  return true;
}

double pathLength(const std::vector<Pose2D> & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return total;
}

}  // namespace

int main()
{
  const CostGrid grid = makeMap();
  const Pose2D start{-4.0, -4.0, 0.0};
  const Pose2D goal{4.0, 4.0, 0.0};

  const std::vector<std::string> names = Registry::instance().availableNames();
  std::printf("Registered algorithms (%zu):\n", names.size());
  for (const std::string & n : names) {std::printf("  - %s\n", n.c_str());}
  std::printf("\n");

  check(names.size() >= 6, "expected at least 6 registered algorithms");

  for (const std::string & name : names) {
    auto planner = Registry::instance().create(name);
    check(planner != nullptr, "factory produced a planner for " + name);
    if (!planner) {continue;}

    ParamMap params;
    params["cost_scale"] = 1.0;
    params["snap_radius"] = 6.0;
    planner->configure(params);

    const algo_core::PlanResult r = planner->plan(grid, start, goal);
    std::printf("%-16s success=%-3s len=%7.3f m  cost=%8.2f  iter=%6zu  %7.2f ms  %s\n",
      name.c_str(), r.success ? "yes" : "no", pathLength(r.path), r.cost,
      r.iterations, r.planning_time_ms, r.message.c_str());

    check(r.success, name + ": found a path");
    if (!r.success) {continue;}

    check(r.path.size() >= 2, name + ": path has at least two poses");
    check(std::hypot(r.path.front().x - start.x, r.path.front().y - start.y) < 0.5,
      name + ": path starts near the start pose");
    check(std::hypot(r.path.back().x - goal.x, r.path.back().y - goal.y) < 0.5,
      name + ": path ends near the goal pose");

    std::string why;
    check(pathIsFree(grid, r.path, why), name + ": path is collision free (" + why + ")");
  }

  // --- D* Lite incrementality -------------------------------------------------
  std::printf("\nD* Lite incremental check:\n");
  if (auto dstar = Registry::instance().create("d_star_lite")) {
    ParamMap params;
    params["cost_scale"] = 1.0;
    dstar->configure(params);

    const algo_core::PlanResult first = dstar->plan(grid, start, goal);
    check(first.success, "d_star_lite: initial plan succeeds");

    // Block the doorway on a copy and replan on the same instance. The planner
    // must notice the change and produce a different, still valid route.
    CostGrid changed = grid;
    std::vector<uint8_t> data = changed.data();
    const unsigned int nx = changed.nx();
    for (unsigned int y = 90; y <= 110; ++y) {
      data[static_cast<std::size_t>(y) * nx + 100] = 254;
    }
    changed = CostGrid(nx, changed.ny(), changed.resolution(),
        changed.originX(), changed.originY(), std::move(data), 253);

    const algo_core::PlanResult second = dstar->plan(changed, start, goal);
    std::printf("  before doorway block: success=%s len=%.3f m  %.2f ms\n",
      first.success ? "yes" : "no", pathLength(first.path), first.planning_time_ms);
    std::printf("  after  doorway block: success=%s len=%.3f m  %.2f ms\n",
      second.success ? "yes" : "no", pathLength(second.path), second.planning_time_ms);

    check(!second.success, "d_star_lite: correctly reports no path once the only doorway is sealed");

    // Unblock again on the same instance: the repair must recover.
    const algo_core::PlanResult third = dstar->plan(grid, start, goal);
    check(third.success, "d_star_lite: recovers after the obstacle is removed");
    std::string why;
    check(pathIsFree(grid, third.path, why),
      "d_star_lite: recovered path is collision free (" + why + ")");
  } else {
    check(false, "d_star_lite is registered");
  }

  std::printf("\n%s (%d failure%s)\n",
    g_failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED",
    g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
