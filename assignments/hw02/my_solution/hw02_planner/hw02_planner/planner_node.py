"""ROS node: wait for map/start/goal, plan once, publish the results, then drive.

Driving is a tiny state machine run by a 10 Hz timer, so the executor keeps
spinning while the robot moves (we never block inside a callback):

    send waypoint i  ->  wait until its future is done  ->  success? i += 1 : stop

Run (after building and sourcing), e.g.:
    ros2 run hw02_planner planner_node --ros-args -p seed:=0 \
        -p figure_dir:=assignments/hw02/my_solution/figures -p run_name:=mixed
"""
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from hw02_sim.map_server import MAP_QOS          # reliable + transient-local
from hw02_sim.robot_client import RobotClient
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .grid import GridMap
from .planner import MAX_EDGE_LENGTH, NEIGHBOURS_PER_NODE, SAMPLE_SCHEDULE, plan


class PlannerNode(Node):
    def __init__(self):
        super().__init__('hw02_planner', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.seed = self.declare_parameter('seed', 0).value
        self.figure_dir = self.declare_parameter('figure_dir', '').value
        self.run_name = self.declare_parameter('run_name', 'run').value

        # Inputs (latched topics, so late subscription still receives them).
        self.map_msg = self.start_msg = self.goal_msg = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(PoseStamped, '/hw02/start', self.on_start, MAP_QOS)
        self.create_subscription(PoseStamped, '/hw02/goal', self.on_goal, MAP_QOS)

        # Outputs.
        self.cspace_pub = self.create_publisher(OccupancyGrid, '/hw02/cspace', MAP_QOS)
        self.roadmap_pub = self.create_publisher(MarkerArray, '/hw02/roadmap', MAP_QOS)
        self.path_pub = self.create_publisher(PathMsg, '/hw02/planned_path', MAP_QOS)
        self.status_pub = self.create_publisher(String, '/hw02/planner_status', MAP_QOS)

        self.robot = RobotClient(self)
        self.state = 'waiting_for_inputs'
        self.result = None
        self.waypoints = []         # what we still have to drive to (start excluded)
        self.next_index = 0
        self.future = None
        self.create_timer(0.1, self.step)

    # ---- input callbacks ---------------------------------------------------
    def on_map(self, msg):
        self.map_msg = msg

    def on_start(self, msg):
        self.start_msg = msg

    def on_goal(self, msg):
        self.goal_msg = msg

    # ---- main loop ---------------------------------------------------------
    def step(self):
        if self.state == 'waiting_for_inputs':
            if self.map_msg and self.start_msg and self.goal_msg:
                self.plan_route()
        elif self.state == 'executing':
            self.drive()

    def plan_route(self):
        self.set_status('planning')
        raw = GridMap.from_msg(self.map_msg)
        start = (self.start_msg.pose.position.x, self.start_msg.pose.position.y)
        goal = (self.goal_msg.pose.position.x, self.goal_msg.pose.position.y)
        self.get_logger().info(
            f'Planning {start} -> {goal}, seed={self.seed}, samples={SAMPLE_SCHEDULE}, '
            f'k={NEIGHBOURS_PER_NODE}, max edge={MAX_EDGE_LENGTH} m')

        self.result = plan(raw, start, goal, seed=self.seed)
        for number, attempt in enumerate(self.result.attempts, 1):
            self.get_logger().info(
                f'  attempt {number}: {attempt.samples_drawn} samples -> {attempt.nodes} nodes, '
                f'{attempt.edges} edges, {attempt.seconds:.2f} s, path={attempt.found_path}')

        self.publish_cspace(self.result.cspace)
        self.publish_roadmap(self.result.roadmap)
        self.save_figures(raw, start, goal)

        if not self.result.found:
            # Leave the robot where it is. The node keeps spinning so the observer
            # can see that it stays still.
            self.get_logger().warn(f'No path: {self.result.problem}')
            self.set_status('no_path_found')
            self.state = 'done'
            self.save_summary('no_path_found')
            return

        self.get_logger().info(f'Path found: {len(self.result.waypoints)} points, '
                               f'{self.result.length:.2f} m')
        self.publish_path(self.result.waypoints)
        self.set_status('path_found')
        self.waypoints = self.result.waypoints[1:]   # the robot is already at the start
        self.state = 'executing'
        self.set_status('executing')

    def drive(self):
        if self.future is None:                      # nothing in flight: send the next waypoint
            if not self.robot.ready():
                self.get_logger().info('Waiting for the go-to helper...', throttle_duration_sec=2.0)
                return
            x, y = self.waypoints[self.next_index]
            self.get_logger().info(f'Waypoint {self.next_index + 1}/{len(self.waypoints)}: '
                                   f'({x:.2f}, {y:.2f})')
            self.future = self.robot.go_to(x, y)
            return

        if not self.future.done():                   # still moving
            return

        reply = self.future.result()
        self.future = None
        if not reply.success:                        # stop the whole sequence on any failure
            self.get_logger().error(f'Waypoint {self.next_index + 1} failed: {reply.message}')
            self.finish('failed')
            return
        self.next_index += 1
        if self.next_index == len(self.waypoints):
            self.get_logger().info('Goal reached.')
            self.finish('completed')

    def finish(self, status):
        self.state = 'done'
        self.set_status(status)
        self.save_summary(status)

    def cancel_motion(self):
        try:
            self.robot.cancel()
        except Exception as error:               # ROS may already be shutting down
            self.get_logger().warn(f'Could not cancel motion: {error}')

    # ---- publishing helpers ------------------------------------------------
    def set_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'status: {text}')

    def publish_cspace(self, cspace):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info                 # same size, resolution and origin
        msg.data = cspace.to_msg_data()
        self.cspace_pub.publish(msg)

    def publish_roadmap(self, roadmap):
        stamp = self.get_clock().now().to_msg()
        nodes = Marker(ns='samples', id=0, type=Marker.SPHERE_LIST, action=Marker.ADD)
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.04
        nodes.color.r, nodes.color.g, nodes.color.b, nodes.color.a = 0.1, 0.4, 1.0, 1.0
        nodes.points = [Point(x=float(x), y=float(y)) for x, y in roadmap.nodes]

        edges = Marker(ns='edges', id=1, type=Marker.LINE_LIST, action=Marker.ADD)
        edges.scale.x = 0.005
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.1, 0.4, 1.0, 0.4
        for a, b in roadmap.edges():
            for x, y in (roadmap.nodes[a], roadmap.nodes[b]):
                edges.points.append(Point(x=float(x), y=float(y)))

        for marker in (nodes, edges):
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.pose.orientation.w = 1.0
        self.roadmap_pub.publish(MarkerArray(markers=[nodes, edges]))

    def publish_path(self, waypoints):
        msg = PathMsg()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    # ---- report material ---------------------------------------------------
    def output_prefix(self):
        if not self.figure_dir:
            return None
        folder = Path(self.figure_dir)
        folder.mkdir(parents=True, exist_ok=True)
        return folder / f'{self.run_name}_seed{self.seed}'

    def save_figures(self, raw, start, goal):
        prefix = self.output_prefix()
        if prefix is None:
            return
        from .plotting import save_cspace_comparison, save_plan_figure
        title = f'{self.run_name}, seed {self.seed}'
        save_plan_figure(f'{prefix}_plan.png', raw, self.result, start, goal, title)
        save_cspace_comparison(f'{prefix}_cspace.png', raw, self.result.cspace, title)
        self.get_logger().info(f'Figures saved to {prefix}_*.png')

    def save_summary(self, outcome):
        prefix = self.output_prefix()
        if prefix is None:
            return
        result = self.result
        summary = {
            'run': self.run_name, 'seed': self.seed, 'outcome': outcome,
            'sample_schedule': list(SAMPLE_SCHEDULE), 'neighbours_per_node': NEIGHBOURS_PER_NODE,
            'max_edge_length_m': MAX_EDGE_LENGTH,
            'attempts': [vars(a) for a in result.attempts],
            'planning_seconds': round(result.total_seconds, 3),
            'path_length_m': round(result.length, 3) if result.found else None,
            'waypoints': len(result.waypoints) if result.found else 0,
            'waypoints_reached': self.next_index,
        }
        Path(f'{prefix}_summary.json').write_text(json.dumps(summary, indent=2))


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cancel_motion()                         # never leave a goal running
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)   # let the cancel request go out
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
