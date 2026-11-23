# HW02: configuration space, PRM and A*

The implementation is in `hw02_planner/`. It reads the supplied map, start and goal,
constructs conservative configuration space, builds a probabilistic roadmap, runs
A* on that graph and executes the resulting waypoints through `RobotClient`.
There are no layout-specific planning branches or hardcoded obstacle coordinates.

## Validation status

The implementation has not been executed or built. Readiness, simulation runs,
public-observer results and recordings are pending. No experimental success or
timing results are claimed in this report. Generated figures and measurements
must be added after performing the runs below.

## Dependencies

Use the course's Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic environment and the
HW02 environment installation guide. The package declares `ament_python`, `rclpy`,
`hw02_sim`, `hw02_interfaces`, `geometry_msgs`, `nav_msgs`, `std_msgs`,
`visualization_msgs`, NumPy, SciPy and Matplotlib. SciPy supplies the distance
transform and nearest-neighbor tree only. PRM construction, collision checking,
A* search and waypoint sequencing are implemented here.

## Build

All commands in this report start in `assignments/hw02/` unless stated otherwise.
Use the same `ROS_DOMAIN_ID` and, when applicable, `GZ_PARTITION` in every terminal.
Deactivate Conda/virtual environments when using the course's apt-installed ROS.

Build the supplied environment first:

```bash
source /opt/ros/jazzy/setup.bash
cd environment
colcon build --symlink-install --base-paths src --executor sequential
source install/setup.bash
cd ..
```

Build the student package:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
colcon --log-base my_solution/log build \
  --symlink-install --base-paths my_solution/hw02_planner \
  --build-base my_solution/build --install-base my_solution/install
source my_solution/install/setup.bash
```

In every new terminal, source these overlays from `assignments/hw02/`:

```bash
source /opt/ros/jazzy/setup.bash
source environment/install/setup.bash
source my_solution/install/setup.bash
```

## Readiness

With a fresh `wide` simulation running in terminal A:

```bash
ros2 launch hw02_sim simulation.launch.py layout:=wide rviz:=true
```

In terminal B:

```bash
ros2 run hw02_sim readiness_demo
python3 my_solution/read_map.py
python3 my_solution/move_demo.py
```

Record the actual output before submission:

| Check | Result |
| --- | --- |
| ROS talker/listener | Not run |
| Environment and planner builds | Not run |
| Shared readiness checks | Not run |
| Raw-map example and coordinate agreement | Not run |
| Movement example, return to start, clear contacts | Not run |

Stop and restart the simulation after readiness/movement checks and before every
demonstration. A restart restores the initial pose and clears latched contacts.

## Run a demonstration

Terminal A (replace `wide` with `mixed` or `blocked` as required):

```bash
ros2 launch hw02_sim simulation.launch.py layout:=wide rviz:=true
```

Headless simulation is available with `gui:=false` instead of `rviz:=true`.

Terminal B: start the independent observer **before** the planner:

```bash
ros2 run hw02_sim check_navigation --layout wide --timeout 300
```

Terminal C:

```bash
ros2 run hw02_planner planner --ros-args \
  -p seed:=0 -p output_dir:=my_solution/results/wide/seed_0
```

Use matching layout names for simulation and observer. Only the output directory
contains a layout label in the planner command; it never affects planning. Repeat
with seeds 1 and 2, using separate output directories and a fresh simulation each
time. Demonstrate `blocked` with seed 0 and its own output directory.

The node stays alive after completion and after bounded no-path planning. For a
blocked run, wait for the log confirming three stationary simulated seconds.
Save the observer's separate PASS/FAIL output. Stop the planner with Ctrl+C before
stopping the simulation; interruption cancels any pending movement.

## Algorithm

### Configuration space

The raw occupancy grid is not inflated. Only a raw value of zero is considered
free; every nonzero value, including unknown, is occupied. The map is padded by an
occupied ring so the exterior participates in obstacle-distance calculations.

The robot's complete physical radius is 0.25 m. The required safety margin is
0.05 m, giving a planning radius of 0.30 m. Since heading does not change the
circular footprint, the search configuration is `(x, y)`.

The Euclidean distance transform measures distance between cell centers. Merely
comparing it with 0.30 m would ignore cell area. Each occupied square and each
candidate square extends by at most half a cell diagonal from its center. Thus
a whole candidate square is marked free only when its distance to the nearest
occupied cell center is strictly greater than:

```text
planning_radius + sqrt(2) * resolution
```

At the supplied 0.025 m resolution, this threshold is approximately 0.33536 m.
This is a conservative raster bound, rather than a change to the required robot
radius. It also accounts for occupied boundary cells and the padded exterior.

World/grid conversion uses map resolution, origin translation and origin yaw.
Column is local X; row is local Y. Saved figures use transformed cell corners,
so increasing rows correspond to increasing local Y rather than a flipped image.
The published configuration-space map preserves the original metadata.

### PRM

| Parameter | Default | Meaning |
| --- | --- | --- |
| `seed` | 0 | Nonnegative NumPy random seed |
| `samples` | 1000 | Random samples in the first attempt |
| `neighbors` | 20 | Nearest neighbors considered for each vertex |
| `connection_radius` | 1.5 m | Maximum candidate connection length |
| `attempts` | 3 | Maximum number of independent roadmap attempts |
| `growth` | 1.5 | Sample multiplier after each unsuccessful attempt |
| `planning_radius` | 0.30 m | Minimum permitted inflation radius |
| `startup_timeout` | 30 wall seconds | Maximum initial wait for required inputs |

These are ROS parameters; for example, `-p samples:=1500 -p neighbors:=25`.

Free cells are selected uniformly and points are sampled uniformly inside their
interiors. The received start and goal are vertices 0 and 1. A nearest-neighbor
tree finds candidate pairs, which are deduplicated and sorted. A candidate is
accepted as an undirected edge only after the complete segment passes collision
checking. The neighbor limit restricts candidate proposals; the final undirected
degree can exceed this limit because other vertices also propose connections.

Each failed attempt rebuilds the graph using the same advancing seeded random
generator. The defaults try 1000, 1500 and 2250 samples, at most 4750 random
samples in total across three roadmaps. Sampling count, candidate connections
and retry count are finite. A successful attempt ends the retry loop immediately.

### Entire-edge collision checking

Segments are split at every vertical and horizontal grid-line crossing. Every
crossing/end point and one interior point of every resulting interval is checked.
An interval lies entirely inside a cell or on a cell boundary. At boundary points
both adjacent cells are checked; at corners all four incident cells are checked.
This covers every touched cell, including grid-aligned and zero-length segments.
Segments touching occupied cells or the map exterior are rejected.

### A*

The adjacency list contains accepted PRM edges, with Euclidean lengths as costs.
A* uses a heap, best-known costs, parent pointers and Euclidean distance to the
goal as its heuristic. Outdated heap entries are skipped. The heuristic is
admissible and consistent because graph edge costs are Euclidean distances.
A* therefore finds a shortest route in the sampled roadmap. This does not imply
the shortest continuous-space route. Parent pointers reconstruct an ordered
start-to-goal path; a disconnected goal produces no partial execution route.

### ROS and movement

Planning and figure rendering run on a worker thread while the ROS executor
continues processing physics streams, action replies and watchdogs. The node
enables simulated time. Static subscriptions and all four required output topics
use reliable, transient-local QoS:

| Output | ROS type |
| --- | --- |
| `/hw02/cspace` | `nav_msgs/msg/OccupancyGrid` |
| `/hw02/roadmap` | `visualization_msgs/msg/MarkerArray` |
| `/hw02/planned_path` | `nav_msgs/msg/Path` |
| `/hw02/planner_status` | `std_msgs/msg/String` |

Statuses are `planning`, `path_found`, `executing`, `completed`,
`no_path_found` and `failed`. The roadmap markers include samples, accepted edges
and start/goal. An unsuccessful search publishes an empty path, never a fabricated
route. The path topic is for visualization; movement uses the action helper.

Before movement, the entire selected route is validated again. Waypoint 0 is the
supplied start, so execution begins with waypoint 1. Before each action, the
segment from the measured position to the next waypoint is also checked. Exactly
one `go_to` request is active at a time. Its asynchronous result and measured
arrival are checked before advancing. Any action failure ends the sequence.

The node requires fresh pose, velocity and collision-monitor messages and an
advancing simulation clock. It rejects a moving robot or a robot displaced from
the supplied start before execution. Changed map/targets, latched contact, stale
physics streams, clock reset/stall or invalid inputs cause failure and action
cancellation. The planner does not publish competing velocity commands.

Completion requires measured goal error at most 0.10 m and low velocity for two
simulated seconds. A no-path run verifies the robot stays within 0.03 m of its
start and remains stopped for three simulated seconds. The independent public
observer must still be run; internal checks and helper success do not replace it.

## Output files and experimental results

Each output directory contains:

- `planning.png`: physical map beside configuration space, samples, accepted
  roadmap edges and any selected path. A no-path result displays only the graph.
- `planning.npz`: received raw map, inflated grid, map geometry, graph vertices,
  accepted edges and ordered path, for reproducibility.
- `metrics.json`: settings, attempts/retries, planning wall time, graph sizes,
  path length, waypoint outcomes, observed goal error and terminal hold result.

The PNG/NPZ show the final attempt; metrics retain statistics for every attempt.
Planning time includes configuration-space construction, roadmap building and
graph search, but excludes plotting, ROS input waiting and movement.

| Layout | Seed | Samples / neighbors / retries | Planning time | Nodes / edges | Path length | Execution / public observer |
| --- | --- | --- | --- | --- | --- | --- |
| wide | 0 | Pending | Pending | Pending | Pending | Not run |
| wide | 1 | Pending | Pending | Pending | Pending | Not run |
| wide | 2 | Pending | Pending | Pending | Pending | Not run |
| mixed | 0 | Pending | Pending | Pending | Pending | Not run |
| mixed | 1 | Pending | Pending | Pending | Pending | Not run |
| mixed | 2 | Pending | Pending | Pending | Pending | Not run |
| blocked | 0 | Pending | Pending | Pending | No route expected | Not run |

After running, link the generated figures for all layouts here, fill the table
from measured results and add the independent observer outputs. Record one short
demonstration per layout if following the local assignment instructions; keep
large recordings outside Git and add accessible links.

### Physical map versus configuration space

For ideal continuous geometry, two pillars require a surface-to-surface gap
greater than 0.60 m to preserve positive clearance for the planned robot center.
A pillar of physical radius 0.40 m expands to radius 0.70 m. Rasterization and
whole-cell certification can close slightly larger gaps too. The supplied
`mixed` room is intended to retain different openings at its two pillar columns,
requiring a change of direction; `blocked` closes every crossing after inflation.
Compare those expectations against the actual saved figures after running.

### Sampling limitations

`no_path_found` means no route in the finite sampled roadmap. Sparse sampling,
too few neighbors, a short connection radius or conservative inflation can all
disconnect a roadmap even when continuous free space has a passage. Increasing
the documented budget can improve coverage, but a failed finite PRM is not an
impossibility proof. The supplied `blocked` layout is a known demonstration of a
closed barrier; the planner still determines its outcome from the received grid,
without inspecting the layout name.

## Submission

Submit the package, this completed report and required figures under
`my_solution/`. Do not commit colcon build/install/log directories or caches.
Follow the repository's `hw02` branch and PR submission process. Include all
submitting students' names in the PR title and prepare for the live explanation
of inflation, edge coverage, PRM parameters, A*, waypoint sequencing and failures.
