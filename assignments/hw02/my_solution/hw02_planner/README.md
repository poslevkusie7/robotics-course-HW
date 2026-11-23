# hw02_planner

Configuration space → PRM → A* → one `go_to` per waypoint.

| File | What it does |
| --- | --- |
| `grid.py` | OccupancyGrid ↔ numpy, world ↔ cell conversion, "is this point free?" |
| `cspace.py` | Grows every obstacle cell by the 0.30 m planning radius |
| `prm.py` | Samples free points, connects k nearest neighbours, checks every edge along its full length |
| `astar.py` | A* on the roadmap (Euclidean edge cost and heuristic) |
| `planner.py` | Ties the above together, with the retry policy. No ROS, so it is testable offline |
| `planner_node.py` | ROS node: inputs, status/visual outputs, waypoint-by-waypoint driving |
| `plotting.py` | Report figures |

Settings live at the top of `planner.py`: sample schedule `1000 → 2000 → 5000`,
`k = 12` neighbours, maximum edge `1.5 m`. Random generator seed = (`seed`, attempt).

## Build (macOS / RoboStack, from the repository root)

```bash
pixi shell --manifest-path ~/ros2-jazzy/pixi.toml
```
```bash
source assignments/hw02/environment/install/setup.zsh
cd assignments/hw02/my_solution && colcon build --symlink-install --base-paths hw02_planner && cd ../../..
```

## Run one demonstration (three terminals, each sourced as above plus
`source assignments/hw02/my_solution/install/setup.zsh`)

```bash
ros2 launch hw02_sim simulation.launch.py layout:=mixed gui:=false
```
```bash
ros2 run hw02_sim check_navigation --layout mixed --timeout 300
```
```bash
ros2 run hw02_planner planner_node --ros-args -p seed:=0 -p run_name:=mixed -p figure_dir:=assignments/hw02/my_solution/figures
```

Restart the simulation for every layout/seed. With `figure_dir` set, the node saves
`<run>_seed<N>_plan.png`, `<run>_seed<N>_cspace.png` and `<run>_seed<N>_summary.json`.

## Tests

```bash
PYTHONPATH=assignments/hw02/my_solution/hw02_planner:$PYTHONPATH python -m pytest -q assignments/hw02/my_solution/hw02_planner/test
```
