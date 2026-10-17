#!/usr/bin/env python3
"""Xbox intent + latched stop gate. All watchdogs use wall monotonic time."""
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mobile_openarm_shared_control.msg import HumanIntent


def steering(value, deadzone, maximum):
    value = max(-1.0, min(1.0, value))
    return 0.0 if abs(value) <= deadzone else math.copysign(value * value, value) * maximum


class SharedControl(Node):
    def __init__(self):
        super().__init__('shared_control')
        defaults = {
            'enable_button': 4, 'stop_button': -1, 'steering_axis': 0,
            'deadzone': 0.1, 'angular_sign': 1.0,
            'max_linear': 0.3, 'max_angular': 0.65,
            'joy_timeout': 0.5, 'command_timeout': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        c = self.cfg
        if (c['enable_button'] < 0 or c['steering_axis'] < 0 or c['stop_button'] < -1
                or c['stop_button'] == c['enable_button']
                or not 0 <= c['deadzone'] < 1
                or c['angular_sign'] not in (-1.0, 1.0)
                or any(not math.isfinite(c[k]) or c[k] <= 0 for k in
                       ('max_linear', 'max_angular', 'joy_timeout', 'command_timeout'))):
            raise ValueError('Invalid shared-control mapping, limits or timeouts')
        self.epoch = 0
        self.stopped = True
        self.reason = 'STARTUP: release LB, then call /shared_control/resume'
        self.joy_at = -math.inf
        self.command_at = -math.inf
        self.command = Twist()
        self.enable_held = False
        self.stop_held = False
        self.angular = 0.0
        self.armed = False
        self.goals = {}
        self.goal_key = ()
        self.cancel_pending = {}
        self.cancel_clients = {}
        self.output = self.create_publisher(Twist, '/shared_control/cmd_vel', 1)
        self.intent = self.create_publisher(HumanIntent, '/shared_control/intent', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/shared_control/status', latched)
        self.create_subscription(Joy, '/joy', self.receive_joy, 1)
        self.create_subscription(Twist, '/shared_control/nav_cmd_vel', self.receive_command, 1)
        for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path'):
            self.create_subscription(
                GoalStatusArray, f'/{action}/_action/status',
                lambda msg, action=action: self.receive_goals(action, msg), latched)
        for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path',
                       'spin', 'backup', 'drive_on_heading', 'wait'):
            self.cancel_clients[action] = self.create_client(CancelGoal, f'/{action}/_action/cancel_goal')
        self.create_service(Trigger, '/shared_control/stop', self.stop_service)
        self.create_service(Trigger, '/shared_control/resume', self.resume_service)
        # Node's ROS clock is simulated; explicitly use a steady timer for watchdogs.
        self.wall_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.wall_clock)
        self.last_status = None

    def receive_joy(self, msg):
        c = self.cfg
        required_button = max(c['enable_button'], c['stop_button'])
        if (len(msg.buttons) <= required_button or len(msg.axes) <= c['steering_axis']
                or any(not math.isfinite(x) for x in msg.axes)):
            self.latch_stop('INVALID_JOY')
            self.joy_at = -math.inf
            return
        self.joy_at = time.monotonic()
        self.enable_held = bool(msg.buttons[c['enable_button']])
        self.stop_held = c['stop_button'] >= 0 and bool(msg.buttons[c['stop_button']])
        self.angular = steering(msg.axes[c['steering_axis']], c['deadzone'], c['max_angular']) * c['angular_sign']
        if self.stop_held:
            self.latch_stop('STOP_BUTTON')
        if not self.enable_held and not self.stopped:
            self.armed = True

    def receive_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.latch_stop('INVALID_NAV_COMMAND')
            return
        if not self.stopped and self.goal_key:
            self.command = msg
            self.command_at = time.monotonic()

    def receive_goals(self, action, msg):
        # CANCELING (3) is no longer an active navigation session.
        self.goals[action] = tuple(sorted(bytes(s.goal_info.goal_id.uuid)
                                          for s in msg.status_list if s.status in (1, 2)))
        outer = tuple((a, ids) for a, ids in sorted(self.goals.items())
                      if a != 'follow_path' and ids)
        key = outer
        if not key and self.goals.get('follow_path'):
            key = (('follow_path', self.goals['follow_path']),)
        if key != self.goal_key:
            self.goal_key = key
            self.epoch += 1
            self.armed = False
            self.command_at = -math.inf
        if self.stopped and key:
            self.cancel_navigation()

    def cancel_navigation(self):
        for action, client in self.cancel_clients.items():
            pending = self.cancel_pending.get(action)
            if client.service_is_ready() and (pending is None or pending.done()):
                # Zero stamp and zero UUID cancel every goal on this action server.
                self.cancel_pending[action] = client.call_async(CancelGoal.Request())

    def latch_stop(self, reason):
        if not self.stopped:
            self.epoch += 1
            self.stopped = True
            self.reason = reason
            self.armed = False
            self.command_at = -math.inf
            self.cancel_navigation()
        self.output.publish(Twist())

    def stop_service(self, request, response):
        self.latch_stop('STOP_SERVICE')
        self.cancel_navigation()
        response.success = True
        response.message = 'Stopped and navigation cancellation requested. Resume explicitly, then send a new goal.'
        return response

    def resume_service(self, request, response):
        fresh = time.monotonic() - self.joy_at <= self.cfg['joy_timeout']
        if not fresh or self.enable_held or self.stop_held or self.goal_key:
            response.success = False
            response.message = 'Need fresh valid /joy, LB/stop released, and no active navigation goal.'
        else:
            self.stopped = False
            self.reason = ''
            self.epoch += 1
            self.armed = True
            self.command_at = -math.inf
            response.success = True
            response.message = 'Ready. Send a Nav2 goal; LB + horizontal stick enables influence.'
        return response

    def tick(self):
        now = time.monotonic()
        if now - self.joy_at > self.cfg['joy_timeout']:
            self.latch_stop('JOY_TIMEOUT')
        permitted = not self.stopped and bool(self.goal_key)
        active = permitted and self.armed and self.enable_held and self.angular != 0.0
        self.intent.publish(HumanIntent(
            epoch=self.epoch, permitted=permitted, active=active,
            linear=self.cfg['max_linear'], angular=self.angular))
        # Forward each command unchanged: a post-DWB velocity clamp would change
        # the collision-checked trajectory. Overspeed commands are stopped instead.
        out = Twist()
        if permitted and now - self.command_at <= self.cfg['command_timeout']:
            if (abs(self.command.linear.x) > self.cfg['max_linear'] + 1e-6
                    or abs(self.command.angular.z) > self.cfg['max_angular'] + 1e-6
                    or abs(self.command.linear.y) > 1e-6):
                self.latch_stop('NAV_COMMAND_OUT_OF_BOUNDS')
            else:
                out = self.command
        self.output.publish(out)
        state = 'STOPPED: ' + self.reason if self.stopped else ('NAVIGATING' if permitted else 'READY')
        if state != self.last_status:
            self.status.publish(String(data=state))
            self.get_logger().info(state)
            self.last_status = state


def main():
    # ros2 launch may signal the process group and this process separately.
    # Keep the ROS context alive until the final zero command has been sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = SharedControl()
    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.latch_stop('SHUTDOWN')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
