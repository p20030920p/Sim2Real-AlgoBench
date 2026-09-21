# Roadmap and open questions

## Next

- [ ] JPS: implement the pruning correctly
- [ ] Sampling planners: RRT, RRT\*, Informed RRT\*, RRT-Connect
- [ ] Controller adapters: DWA, APF, RPP, LQR, MPC
- [ ] Hardware bring-up and calibration record

## The JPS discrepancy

JPS is registered but does not work offline: it returns `NO_VALID_PATH` on the race map, expanding 5
cells, where A\* finds a route with the same cost model. Its pruning is therefore incorrect. Do not
use it for reported results.

It is worth a second look that JPS *does* complete the full task in the recorded clips, at the
greatest distance of the seven. The two do not plan on the same thing: the offline dump converts the
saved map with map_server's trinary rule, while the stack plans on the inflated global costmap, and
JPS's pruning survives one and not the other. Until that is understood, its search should not be
trusted, and the offline table in the README is the one to read.

The offline table is produced by `algo_plan_dump`, which is also what
[`render_planning_demo.py`](../tools/render_planning_demo.py) consumes; that is why the search figure
has to be rendered from the same dump as the table, or the two disagree.

## Adding an algorithm

See [`ALGORITHM_PLUGINS.md`](ALGORITHM_PLUGINS.md).
