#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Compose a nav2_params file whose global planner is a registered algorithm.

The race stack in ``race_navigation/config/nav2_params.yaml`` plans with Nav2's
own Theta* under the plugin id ``GridBased``. To run the *same* scenario with a
different algorithm, only that one block has to change: every other node,
costmap, controller and behaviour tree stays identical, so a run with A* and a
run with D* Lite differ in nothing but the global planner.

The algorithm is registered under the id ``GridBased`` rather than its own
``P1_astar``-style id on purpose. Nav2's stock behaviour tree resolves the
planner through its planner selector and falls back to the first entry in
``planner_plugins``; reusing that id means no custom behaviour tree is needed
and ``competition.launch.py`` is untouched.

Usage:
    python3 tools/make_planner_params.py --algorithm astar --out /tmp/nav2_astar.yaml
    python3 tools/make_planner_params.py --algorithm nav2_theta_star --out /tmp/nav2_base.yaml
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_BASE = os.path.join(
    REPO, "src", "race_navigation", "config", "nav2_params.yaml")
DEFAULT_REGISTRY = os.path.join(
    REPO, "src", "algo_bringup", "config", "algo_registry.yaml")

# The plugin id the stock behaviour tree falls back to. Keeping it means
# NavigateToPose needs no planner_selector and no hand-written BT.
PLUGIN_ID = "GridBased"

# Parameters the registry forwards to algo_core through the plugin, mirroring
# algo_bringup/registry.py:FORWARDED_PARAMS.
FORWARDED = ("cost_scale", "snap_radius", "weight",
             "allow_diagonal", "remove_collinear", "smooth")


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def find_planner(registry: dict, algorithm: str) -> dict:
    for entry in registry.get("planners", []):
        if entry.get("algorithm") == algorithm or entry.get("id") == algorithm:
            return entry
    names = [e.get("algorithm") for e in registry.get("planners", [])]
    raise SystemExit(
        f"algorithm {algorithm!r} is not in the registry; have {names}")


def planner_block(entry: dict, publish_expanded: bool) -> dict:
    """The ``GridBased`` parameter block for one registry entry."""
    if entry.get("external"):
        # A Nav2 plugin referenced by its real class name.
        return {"plugin": entry["algorithm"]}

    block = {
        "plugin": "algo_nav2_plugins/GridPlanner",
        "algorithm": entry["algorithm"],
        "publish_expanded": bool(publish_expanded),
    }
    for key in FORWARDED:
        if key in entry.get("params", {}):
            block[key] = entry["params"][key]
    return block


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--algorithm", required=True,
                    help="algorithm name or registry id, e.g. astar / P1_astar")
    ap.add_argument("--out", required=True, help="params file to write")
    ap.add_argument("--base", default=DEFAULT_BASE, help="nav2_params.yaml to start from")
    ap.add_argument("--registry", default=DEFAULT_REGISTRY)
    ap.add_argument("--publish-expanded", action="store_true",
                    help="have the plugin publish its expanded cells")
    args = ap.parse_args()

    params = load_yaml(args.base)
    registry = load_yaml(args.registry)
    entry = find_planner(registry, args.algorithm)

    if "planner_server" not in params:
        raise SystemExit(f"{args.base} has no planner_server section")

    server = params["planner_server"]["ros__parameters"]
    server["planner_plugins"] = [PLUGIN_ID]
    server[PLUGIN_ID] = planner_block(entry, args.publish_expanded)

    with open(args.out, "w", encoding="utf-8") as handle:
        yaml.safe_dump(params, handle, sort_keys=False, default_flow_style=False)

    print(f"wrote {args.out}")
    print(f"  planner id {PLUGIN_ID!r} -> {entry['algorithm']}")
    if entry.get("params"):
        forwarded = {k: v for k, v in entry["params"].items() if k in FORWARDED}
        if forwarded:
            print(f"  params {forwarded}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
