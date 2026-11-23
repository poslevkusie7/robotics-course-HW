"""ROS integration and a nonblocking, one-waypoint-at-a-time execution state machine."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from hw02_sim.robot_client import RobotClient
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from .artifacts import save_planning_artifacts
from .roadmap import PlannerSettings, plan


STATIC_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('hw02_planner')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        defaults = asdict(PlannerSettings())
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = PlannerSettings(**{
            name: self.get_parameter(name).value for name in defaults
        })
        self.settings.validate()
        self.declare_parameter('output_dir', '')
        self.declare_parameter('startup_timeout', 30.0)
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        if not math.isfinite(self.startup_timeout) or self.startup_timeout <= 0:
            raise ValueError('startup_timeout must be positive and finite')
        directory = self.get_parameter('output_dir').value
        self.output = Path(directory or f'my_solution/results/seed_{self.settings.seed}').expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)

        self.map_message = self.start_message = self.goal_message = None
        self.map_signature = None
        self.pose = self.speed = None
        self.pose_received = self.speed_received = self.contact_received = 0.0
        self.collision = False
        self.state = 'waiting'
        self.status = 'planning'
        self.started = False
        self.began = time.monotonic()
        self.last_clock = 0
        self.clock_changed = self.began
        self.result = None
        self.planning_future = self.export_future = self.movement_future = None
        self.waypoint_index = 1
        self.waypoint_results = []
        self.execution_started = None
        self.execution_seconds = None
        self.hold_started = None
        self.hold_seconds = 0.0
        self.stationary_verified = False
        self.message = 'Waiting for map, targets, fresh physics streams, clock and movement helper'

        self.cspace_publisher = self.create_publisher(OccupancyGrid, '/hw02/cspace', STATIC_QOS)
        self.roadmap_publisher = self.create_publisher(MarkerArray, '/hw02/roadmap', STATIC_QOS)
        self.path_publisher = self.create_publisher(RosPath, '/hw02/planned_path', STATIC_QOS)
        self.status_publisher = self.create_publisher(String, '/hw02/planner_status', STATIC_QOS)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, STATIC_QOS)
        self.create_subscription(PoseStamped, '/hw02/start', self.on_start, STATIC_QOS)
        self.create_subscription(PoseStamped, '/hw02/goal', self.on_goal, STATIC_QOS)
        self.create_subscription(PoseStamped, '/hw02/robot_pose', self.on_pose, 10)
        self.create_subscription(Twist, '/hw02/robot_velocity', self.on_speed, 10)
        self.create_subscription(Bool, '/hw02/collision', self.on_collision, 10)
        self.robot = RobotClient(self)
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='hw02_planning')
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status('planning', self.message)

    def on_map(self, message):
        origin = message.info.origin
        signature = (
            message.header.frame_id, message.info.width, message.info.height,
            message.info.resolution, origin.position.x, origin.position.y, origin.position.z,
            origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w,
            np.asarray(message.data, dtype=np.int8).tobytes(),
        )
        if self.started and signature != self.map_signature:
            self.fail('Map changed during this run; restart before replanning')
            return
        self.map_message, self.map_signature = message, signature

    @staticmethod
    def target_key(message):
        p = message.pose.position
        return message.header.frame_id, p.x, p.y, p.z

    def on_start(self, message):
        if self.started and self.target_key(message) != self.target_key(self.start_message):
            self.fail('Start changed during this run; restart before replanning')
            return
        self.start_message = message

    def on_goal(self, message):
        if self.started and self.target_key(message) != self.target_key(self.goal_message):
            self.fail('Goal changed during this run; restart before replanning')
            return
        self.goal_message = message

    def on_pose(self, message):
        p = message.pose.position
        if message.header.frame_id == 'map' and all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.pose, self.pose_received = message, time.monotonic()

    def on_speed(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if all(math.isfinite(value) for value in values):
            self.speed, self.speed_received = message, time.monotonic()

    def on_collision(self, message):
        self.contact_received = time.monotonic()
        self.collision |= message.data

    def fresh_physics(self):
        now = time.monotonic()
        if self.pose is None or self.speed is None:
            return False
        if any(now - received > 1.0 for received in (
                self.pose_received, self.speed_received, self.contact_received)):
            return False
        stamp = self.pose.header.stamp
        age = self.get_clock().now().nanoseconds - (stamp.sec * 1_000_000_000 + stamp.nanosec)
        # Clock and pose arrive on separate topics; allow one physics publication
        # interval of ordering skew without treating real stale data as fresh.
        return -50_000_000 <= age < 300_000_000

    def stopped(self):
        return (math.hypot(self.speed.linear.x, self.speed.linear.y) < 0.02
                and abs(self.speed.angular.z) < 0.05)

    def position(self):
        p = self.pose.pose.position
        return np.array([p.x, p.y])

    def publish_status(self, status, message):
        self.status = status
        self.message = message
        self.status_publisher.publish(String(data=status))
        self.get_logger().info(f'{status}: {message}')

    def save_metrics(self):
        metrics = {
            'settings': asdict(self.settings),
            'status': self.status,
            'message': self.message,
            'elapsed_wall_seconds': time.monotonic() - self.began,
            'execution_seconds': self.execution_seconds,
            'waypoint_results': self.waypoint_results,
            'collision_observed': self.collision,
            'stationary_hold_sim_seconds': self.hold_seconds,
            'stationary_verified': self.stationary_verified,
            'independent_observer': 'Run check_navigation separately and save its output',
        }
        if self.result is not None:
            last = self.result.attempts[-1]
            metrics.update({
                'planning_seconds': self.result.planning_seconds,
                'attempts': self.result.attempts,
                'retries': len(self.result.attempts) - 1,
                'nodes': len(self.result.nodes),
                'edges': len(self.result.edges),
                'path_length_m': last['path_length_m'],
                'start': self.result.nodes[0].tolist(),
                'goal': self.result.nodes[1].tolist(),
            })
        if self.pose is not None and self.goal_message is not None:
            goal = self.goal_message.pose.position
            metrics['goal_error_m'] = math.hypot(self.pose.pose.position.x - goal.x,
                                                self.pose.pose.position.y - goal.y)
        try:
            temporary = self.output / 'metrics.json.tmp'
            temporary.write_text(json.dumps(metrics, indent=2, allow_nan=False) + '\n')
            temporary.replace(self.output / 'metrics.json')
        except (OSError, ValueError) as error:
            self.get_logger().error(f'Could not save run metrics: {error}')

    def fail(self, reason):
        if self.state == 'failed':
            return
        self.robot.cancel()
        self.state = 'failed'
        self.publish_status('failed', reason)
        if self.execution_started is not None:
            self.execution_seconds = time.monotonic() - self.execution_started
        self.save_metrics()

    def begin_planning(self):
        message = self.map_message
        if message.header.frame_id != 'map':
            raise ValueError('Occupancy map frame must be map')
        for target in (self.start_message, self.goal_message):
            if target.header.frame_id != 'map' or not all(math.isfinite(v) for v in self.target_key(target)[1:]):
                raise ValueError('Start and goal must be finite poses in map')
            if abs(target.pose.position.z) > 1e-6:
                raise ValueError('Start and goal must be 2D poses with z=0')
        if not self.stopped():
            raise ValueError('Robot must be stopped before planning; restart the simulation')
        start = self.start_message.pose.position
        goal = self.goal_message.pose.position
        if np.linalg.norm(self.position() - (start.x, start.y)) > 0.03:
            raise ValueError('Robot is not at the supplied start; restart the simulation')
        info = message.info
        if info.width < 1 or info.height < 1 or len(message.data) != info.width * info.height:
            raise ValueError('Map dimensions do not match occupancy data')
        p = info.origin.position
        if not all(math.isfinite(value) for value in (p.x, p.y, p.z)) or abs(p.z) > 1e-6:
            raise ValueError('Map origin must be finite and planar with z=0')
        q = info.origin.orientation
        if (not all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w))
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                or not math.isclose(q.z*q.z + q.w*q.w, 1.0, abs_tol=1e-5)):
            raise ValueError('Map origin must have a normalized planar orientation')
        yaw = math.atan2(2.0*q.w*q.z, 1.0 - 2.0*q.z*q.z)
        raw = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width).copy()
        self.started = True
        self.state = 'planning'
        self.planning_future = self.worker.submit(
            plan, raw, float(info.resolution), (info.origin.position.x, info.origin.position.y),
            yaw, (start.x, start.y), (goal.x, goal.y), self.settings,
        )
        self.publish_status('planning', f'Building configuration space and PRM with seed {self.settings.seed}')

    def publish_plan(self):
        result = self.result
        grid = deepcopy(self.map_message)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.data = np.where(result.space.blocked, 100, 0).astype(np.int8).ravel().tolist()
        self.cspace_publisher.publish(grid)
        path = RosPath(header=deepcopy(grid.header))
        for x, y in result.path:
            pose = PoseStamped(header=deepcopy(grid.header))
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)

        def marker(identifier, kind, color, scale):
            item = Marker(header=deepcopy(grid.header), ns='student_prm', id=identifier,
                          type=kind, action=Marker.ADD)
            item.pose.orientation.w = 1.0
            item.scale.x = item.scale.y = item.scale.z = scale
            item.color.r, item.color.g, item.color.b, item.color.a = color
            return item

        samples = marker(0, Marker.POINTS, (0.15, 0.45, 0.85, 1.0), 0.035)
        samples.points = [Point(x=float(x), y=float(y), z=0.06) for x, y in result.nodes[2:]]
        edges = marker(1, Marker.LINE_LIST, (0.2, 0.55, 0.8, 0.35), 0.008)
        edges.points = [Point(x=float(result.nodes[index, 0]), y=float(result.nodes[index, 1]), z=0.04)
                        for pair in result.edges for index in pair]
        start = marker(2, Marker.SPHERE_LIST, (0.1, 0.8, 0.2, 1.0), 0.14)
        goal = marker(3, Marker.SPHERE_LIST, (0.9, 0.1, 0.15, 1.0), 0.14)
        for item, point in ((start, result.nodes[0]), (goal, result.nodes[1])):
            item.points = [Point(x=float(point[0]), y=float(point[1]), z=0.1)]
        self.roadmap_publisher.publish(MarkerArray(markers=[
            Marker(header=deepcopy(grid.header), action=Marker.DELETEALL), samples, edges, start, goal,
        ]))

    def poll_planning(self):
        if not self.planning_future.done():
            return
        self.result = self.planning_future.result()
        self.publish_plan()
        self.state = 'saving'
        if len(self.result.path):
            self.publish_status('path_found', f'{len(self.result.path)} path poses; saving planning artifacts')
        else:
            self.publish_status('no_path_found',
                                f'No roadmap path after {len(self.result.attempts)} bounded attempts')
        self.save_metrics()
        self.export_future = self.worker.submit(save_planning_artifacts, self.result, self.output)

    def poll_export(self):
        if not self.export_future.done():
            return
        self.export_future.result()
        if len(self.result.path):
            # Validate the whole selected route once more before issuing any action.
            if not all(self.result.space.edge_is_free(first, second)
                       for first, second in zip(self.result.path[:-1], self.result.path[1:])):
                raise ValueError('Selected route failed final collision validation')
            self.state = 'executing'
            self.execution_started = time.monotonic()
            self.publish_status('executing', 'Sending one waypoint at a time')
        else:
            self.state = 'no_path_found'
            self.hold_started = None
            self.get_logger().info('Keeping node alive; verifying at least 3 stationary simulated seconds')
        self.save_metrics()

    def execute_waypoints(self):
        if self.movement_future is not None:
            if not self.movement_future.done():
                return
            reply = self.movement_future.result()
            target = self.result.path[self.waypoint_index]
            error = float(np.linalg.norm(self.position() - target))
            self.waypoint_results.append({
                'index': self.waypoint_index, 'target': target.tolist(),
                'success': bool(reply.success), 'message': reply.message,
                'measured_error_m': error,
            })
            self.movement_future = None
            if not reply.success:
                self.fail(f'Waypoint {self.waypoint_index} failed: {reply.message}')
                return
            if error > 0.04:
                self.fail(f'Waypoint {self.waypoint_index} action succeeded but measured error is {error:.3f} m')
                return
            self.waypoint_index += 1
            self.save_metrics()
        if self.waypoint_index >= len(self.result.path):
            self.state = 'verifying'
            self.hold_started = None
            return
        if not self.robot.ready():
            self.fail('Movement helper became unavailable')
            return
        target = self.result.path[self.waypoint_index]
        if not self.result.space.edge_is_free(self.position(), target):
            self.fail('Measured robot position has no collision-free segment to the next waypoint')
            return
        self.get_logger().info(f'Waypoint {self.waypoint_index}/{len(self.result.path)-1}: '
                               f'({target[0]:.3f}, {target[1]:.3f})')
        self.movement_future = self.robot.go_to(float(target[0]), float(target[1]))

    def verify_stationary(self, now):
        no_path = self.state == 'no_path_found'
        reference = self.result.nodes[0] if no_path else self.result.nodes[1]
        error = float(np.linalg.norm(self.position() - reference))
        if error > (0.03 if no_path else 0.10):
            self.fail('Robot moved from the start' if no_path else 'Measured robot position is outside goal tolerance')
            return
        if not self.stopped():
            self.hold_started = None
            self.hold_seconds = 0.0
            return
        if self.hold_started is None:
            self.hold_started = now
        self.hold_seconds = (now - self.hold_started) / 1e9
        if self.hold_seconds < (3.0 if no_path else 2.0):
            return
        self.stationary_verified = True
        if not no_path:
            self.state = 'completed'
            self.execution_seconds = time.monotonic() - self.execution_started
            self.publish_status('completed', 'Reached goal and stopped for two simulated seconds without contact')
        else:
            self.get_logger().info('No-path run stayed stationary for three simulated seconds')
        self.save_metrics()

    def tick(self):
        try:
            if self.state in ('failed', 'completed') or self.stationary_verified:
                return
            wall = time.monotonic()
            now = self.get_clock().now().nanoseconds
            if now != self.last_clock:
                if self.started and now < self.last_clock:
                    self.fail('Simulation clock reset; restart the planner with the new world')
                    return
                self.last_clock, self.clock_changed = now, wall
            if self.collision:
                self.fail('Obstacle contact is latched; restart the simulation')
                return
            if self.state == 'waiting':
                ready = (all(message is not None for message in (
                    self.map_message, self.start_message, self.goal_message))
                    and self.robot.ready() and self.fresh_physics() and now > 0
                    and wall - self.clock_changed < 1.0)
                if ready:
                    self.begin_planning()
                elif wall - self.began > self.startup_timeout:
                    self.fail('Startup timed out; run readiness and check map/target QoS and simulation')
                return
            if wall - self.clock_changed > 2.0:
                self.fail('Simulation clock stalled')
                return
            if not self.fresh_physics():
                self.fail('Localization, velocity or contact monitoring became stale')
                return
            if self.state in ('planning', 'saving'):
                start = self.start_message.pose.position
                moved = np.linalg.norm(self.position() - (start.x, start.y)) > 0.03
                if moved or not self.stopped():
                    self.fail('Robot moved during planning; restart the simulation')
                    return
            if self.state == 'planning':
                self.poll_planning()
            elif self.state == 'saving':
                self.poll_export()
            elif self.state == 'executing':
                self.execute_waypoints()
            elif self.state in ('verifying', 'no_path_found'):
                self.verify_stationary(now)
        except Exception as error:
            self.fail(f'{type(error).__name__}: {error}')

    def stop(self):
        try:
            self.robot.cancel()
            if self.state not in ('failed', 'completed') and not self.stationary_verified:
                self.state = 'failed'
                self.publish_status('failed', 'Planner interrupted; pending movement canceled')
                self.save_metrics()
        finally:
            self.worker.shutdown(wait=False, cancel_futures=True)


def main(args=None):
    # Keep the context alive on Ctrl+C/SIGTERM until the action is canceled.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PlannerNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            try:
                node.stop()
                # Give cancellation/late goal acceptance callbacks time to finish.
                deadline = time.monotonic() + 2.0
                while (rclpy.ok() and node.robot.pending is not None
                       and time.monotonic() < deadline):
                    executor.spin_once(timeout_sec=0.05)
            except (rclpy.executors.ExternalShutdownException, RuntimeError):
                pass
            executor.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
