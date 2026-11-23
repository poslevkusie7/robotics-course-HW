# Your HW2 implementation

The student planner is in `hw02_planner/`. See [report.md](report.md) for its
algorithm, parameters, exact build/run commands and pending validation results.
The setup examples below remain unchanged.

This directory contains only two setup examples. Complete the
[installation guide](../environment/README.md) before implementing your planner.

## Run the examples

From the repository root, terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
ros2 launch hw02_sim simulation.launch.py layout:=wide
```

Keep A running. In terminal B, also from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
ros2 run hw02_sim readiness_demo
python3 assignments/hw02/my_solution/read_map.py
python3 assignments/hw02/my_solution/move_demo.py
```

`read_map.py` prints map metadata and saves the **raw**, uninflated map as NPZ and
PNG under ignored `build/hw02/demo_map/`. `move_demo.py` moves to `(1.5, 3)` and
returns to `(1, 3)`, checking measured positions. Both fail with a nonzero exit
code if prerequisites or movement fail. Neither example solves the assignment.

## Use one-destination movement

```python
from hw02_sim.robot_client import RobotClient

robot = RobotClient(your_ros_node)
if robot.ready():
    future = robot.go_to(1.5, 3.0)
```

Keep spinning your node. When `future.done()` is true, inspect
`future.result().success` and `.message`. Call `robot.cancel()` on interruption.
`ready()` checks action-server discovery; run readiness to check the complete setup.

For your own planned route, request one waypoint at a time and wait for a successful
result before requesting the next. Stop on failure. The helper turns and drives
directly; it does not avoid obstacles or execute an entire path for you.

A manual equivalent, after sourcing, is:

```bash
ros2 run hw02_sim go_to --x 1.5 --y 3.0
```

Use clear destinations for manual tests. Restart the simulation before assignment
demonstrations to restore the initial robot pose and clear latched contacts.

See [instructions.md](../instructions.md) for planning, visualization and report
requirements. Commit source, report and required figures, not caches/build products.
