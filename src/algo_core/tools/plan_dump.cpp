// Copyright (c) 2026 p20030920p and zfyyyyy
// SPDX-License-Identifier: MIT
//
// Runs every registered algorithm over a ROS map and dumps what the search did,
// so the README demo can be rendered from the real planner output rather than
// from a second implementation kept in step by hand.
//
// The dump is binary because the expansion order is the bulky part: a Dijkstra
// run on the race map expands tens of thousands of cells, and the animation
// needs them in order.
//
// Layout (little endian):
//   uint32  algorithm count
//   per algorithm:
//     uint32 name length, bytes
//     uint8  success
//     double cost, double planning_time_ms, uint64 iterations
//     uint32 path point count, then count x (double x, double y)
//     uint32 expanded count, then count x int32 cell index

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "algo_core/graph/d_star_lite.hpp"
#include "algo_core/registry.hpp"
#include "algo_core/types.hpp"

namespace
{

/// Minimal binary PGM (P5) reader — enough for a ROS map_server image.
bool readPgm(const std::string & path, int & width, int & height,
  std::vector<uint8_t> & pixels)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {return false;}

  std::string magic;
  in >> magic;
  if (magic != "P5") {
    std::cerr << "not a binary PGM: " << magic << "\n";
    return false;
  }

  auto nextToken = [&in]() -> std::string {
      std::string token;
      char c = 0;
      while (in.get(c)) {
        if (std::isspace(static_cast<unsigned char>(c))) {
          if (!token.empty()) {break;}
          continue;
        }
        if (c == '#') {
          std::string line;
          std::getline(in, line);
          continue;
        }
        token.push_back(c);
      }
      return token;
    };

  width = std::stoi(nextToken());
  height = std::stoi(nextToken());
  const int maxval = std::stoi(nextToken());
  if (maxval != 255) {
    std::cerr << "unsupported maxval " << maxval << "\n";
    return false;
  }
  // nextToken() already consumed the single whitespace byte that follows
  // maxval; the pixel data starts immediately after it. Consuming one more
  // byte here would shift the whole image by a pixel.

  pixels.resize(static_cast<std::size_t>(width) * height);
  in.read(reinterpret_cast<char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  return static_cast<bool>(in);
}

/// Reproduce map_server's trinary conversion so the planner sees the same
/// lethal/free classification the Nav2 plugin would give it.
std::vector<uint8_t> toCostmap(
  const std::vector<uint8_t> & pixels, double free_thresh, double occupied_thresh)
{
  std::vector<uint8_t> out(pixels.size(), 0);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    const double occ = (255.0 - pixels[i]) / 255.0;
    if (occ >= occupied_thresh) {
      out[i] = 254;
    } else if (occ <= free_thresh) {
      out[i] = 0;
    } else {
      out[i] = 255;   // unknown, treated as blocked by CostGrid
    }
  }
  return out;
}

template<typename T>
void put(std::ofstream & out, const T & value)
{
  out.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

void putString(std::ofstream & out, const std::string & text)
{
  put<std::uint32_t>(out, static_cast<std::uint32_t>(text.size()));
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 6) {
    std::cerr <<
      "usage: algo_plan_dump <map.pgm> <out.bin> <start_x> <start_y> "
      "<goal_x> <goal_y> [origin_x origin_y resolution free_thresh occupied_thresh]\n";
    return 2;
  }

  const std::string map_path = argv[1];
  const std::string out_path = argv[2];
  const double start_x = std::stod(argv[3]);
  const double start_y = std::stod(argv[4]);
  const double goal_x = std::stod(argv[5]);
  const double goal_y = std::stod(argv[6]);

  double origin_x = -3.700;
  double origin_y = -6.342;
  double resolution = 0.050;
  double free_thresh = 0.196;
  double occupied_thresh = 0.65;
  if (argc >= 12) {
    origin_x = std::stod(argv[7]);
    origin_y = std::stod(argv[8]);
    resolution = std::stod(argv[9]);
    free_thresh = std::stod(argv[10]);
    occupied_thresh = std::stod(argv[11]);
  }

  int width = 0;
  int height = 0;
  std::vector<uint8_t> pixels;
  if (!readPgm(map_path, width, height, pixels)) {
    std::cerr << "failed to read " << map_path << "\n";
    return 1;
  }

  // PGM rows run top to bottom; the grid's y axis runs bottom to top.
  std::vector<uint8_t> flipped(pixels.size());
  for (int y = 0; y < height; ++y) {
    std::memcpy(&flipped[static_cast<std::size_t>(y) * width],
      &pixels[static_cast<std::size_t>(height - 1 - y) * width],
      static_cast<std::size_t>(width));
  }

  const std::vector<uint8_t> cost = toCostmap(flipped, free_thresh, occupied_thresh);
  const algo_core::CostGrid grid(
    width, height, resolution, origin_x, origin_y, cost, 253);

  const algo_core::Pose2D start{start_x, start_y, 0.0};
  const algo_core::Pose2D goal{goal_x, goal_y, 0.0};

  std::vector<std::string> names = algo_core::Registry::instance().availableNames();
  std::sort(names.begin(), names.end());

  std::ofstream out(out_path, std::ios::binary);
  if (!out) {
    std::cerr << "cannot write " << out_path << "\n";
    return 1;
  }

  put<std::uint32_t>(out, static_cast<std::uint32_t>(names.size()));

  for (const std::string & name : names) {
    auto planner = algo_core::Registry::instance().create(name);
    if (!planner) {continue;}

    algo_core::ParamMap params;
    params["cost_scale"] = 1.0;
    params["snap_radius"] = 6.0;
    planner->configure(params);

    const algo_core::PlanResult result = planner->plan(grid, start, goal);
    std::printf("%-16s success=%-3s expanded=%-7zu %7.2f ms  %s\n",
      name.c_str(), result.success ? "yes" : "no", result.expanded.size(),
      result.planning_time_ms, result.message.c_str());

    putString(out, name);
    put<std::uint8_t>(out, result.success ? 1 : 0);
    put<double>(out, result.cost);
    put<double>(out, result.planning_time_ms);
    put<std::uint64_t>(out, static_cast<std::uint64_t>(result.iterations));

    put<std::uint32_t>(out, static_cast<std::uint32_t>(result.path.size()));
    for (const algo_core::Pose2D & pose : result.path) {
      put<double>(out, pose.x);
      put<double>(out, pose.y);
    }

    put<std::uint32_t>(out, static_cast<std::uint32_t>(result.expanded.size()));
    for (const algo_core::Cell & cell : result.expanded) {
      put<std::int32_t>(out, static_cast<std::int32_t>(cell.y * width + cell.x));
    }
  }

  std::printf("\nwrote %s (%d x %d map)\n", out_path.c_str(), width, height);
  return 0;
}
