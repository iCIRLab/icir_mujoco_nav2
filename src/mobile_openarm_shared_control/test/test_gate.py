"""Real ROS service/topic contract tests in an isolated ROS_DOMAIN_ID.

Use ROS_DOMAIN_ID=146 when running these tests; no hardware or simulator needed.
The node uses_sim_time while /clock is absent, exercising wall-clock watchdogs.
"""
import importlib.util
import math
import os
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from mobile_openarm_shared_control.msg import HumanIntent

spec = importlib.util.spec_from_file_location(
    'shared_control_impl', Path(__file__).parents[1] / 'scripts/shared_control_node.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_steering_curve():
    assert module.steering(0.1, 0.1, 0.65) == 0
    assert module.steering(-0.5, 0.1, 0.65) == pytest.approx(-0.1625)
    assert module.steering(2.0, 0.1, 0.65) == 0.65


def test_live_gate_contract():
    # Never publish synthetic actions onto the user's normal domain (30).
    assert os.environ.get('ROS_DOMAIN_ID') == '146', 'Run with ROS_DOMAIN_ID=146'
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'stop_button:=1'])
    gate = module.SharedControl()
    driver = Node('shared_control_contract_test')
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(driver)
    joy = driver.create_publisher(Joy, '/joy', 1)
    command = driver.create_publisher(Twist, '/shared_control/nav_cmd_vel', 1)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    goals = driver.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status', latched)
    outputs, intents = [], []
    driver.create_subscription(Twist, '/shared_control/cmd_vel', outputs.append, 10)
    driver.create_subscription(HumanIntent, '/shared_control/intent', intents.append, 10)
    resume = driver.create_client(Trigger, '/shared_control/resume')
    stop = driver.create_client(Trigger, '/shared_control/stop')
    cancellations = []

    def cancel(request, response):
        cancellations.append(request)
        return response

    driver.create_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', cancel)
    buttons = [0] * 15
    axis = 0.0
    feed_joy = True
    feed_command = False
    velocity = 0.2

    def pump(seconds):
        until = time.monotonic() + seconds
        next_feed = 0
        while time.monotonic() < until:
            now = time.monotonic()
            if now >= next_feed:
                if feed_joy:
                    joy.publish(Joy(axes=[axis, 0.0], buttons=buttons))
                if feed_command:
                    m = Twist()
                    m.linear.x = velocity
                    command.publish(m)
                next_feed = now + 0.03
            executor.spin_once(timeout_sec=0.01)

    def service(client):
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        for _ in range(40):
            if future.done():
                return future.result()
            pump(0.025)
        raise AssertionError('service timeout')

    def goal(number, status=2):
        item = GoalStatus(status=status)
        item.goal_info.goal_id.uuid = [number] + [0] * 15
        goals.publish(GoalStatusArray(status_list=[item]))
        pump(0.15)

    try:
        pump(0.6)
        assert outputs and outputs[-1].linear.x == 0  # Startup is latched.
        assert service(resume).success
        goal(1)
        feed_command = True
        pump(0.2)
        assert outputs[-1].linear.x == pytest.approx(0.2)
        assert intents[-1].permitted and not intents[-1].active
        buttons[4], axis = 1, 0.5
        pump(0.2)
        assert intents[-1].active
        assert intents[-1].angular == pytest.approx(0.1625)
        old_epoch = intents[-1].epoch
        goal(2)  # Holding LB over a new goal cannot reuse human influence.
        assert intents[-1].epoch != old_epoch and not intents[-1].active
        buttons[4] = 0
        pump(0.15)
        buttons[4] = 1
        pump(0.15)
        assert intents[-1].active
        buttons[4] = 0
        pump(0.15)
        feed_command = False
        pump(0.65)
        assert outputs[-1].linear.x == 0  # Nav command watchdog.
        feed_command = True
        pump(0.2)
        buttons[1] = 1
        pump(0.15)
        assert gate.stopped and outputs[-1].linear.x == 0
        assert cancellations
        assert not any(cancellations[-1].goal_info.goal_id.uuid)
        assert not service(resume).success  # Button still held, active goal.
        buttons[1] = 0
        goal(2, 5)
        assert service(resume).success
        goal(3)
        pump(0.2)
        assert outputs[-1].linear.x > 0
        feed_joy = False
        pump(0.65)  # /clock has never advanced; watchdog must still fire.
        assert gate.stopped and outputs[-1].linear.x == 0
        feed_joy = True
        pump(0.2)
        assert gate.stopped  # Reconnection cannot automatically resume.
        goal(3, 5)
        assert service(resume).success
        goal(4)
        velocity = math.nan
        pump(0.2)
        assert gate.stopped and outputs[-1].linear.x == 0
        goal(4, 5)
        velocity = 0.31
        assert service(resume).success
        goal(5)
        pump(0.2)
        assert gate.stopped and outputs[-1].linear.x == 0  # Overspeed is rejected.
        goal(5, 5)
        velocity = 0.2
        assert service(resume).success
        goal(6)
        axis = math.nan
        pump(0.2)
        assert gate.stopped and outputs[-1].linear.x == 0  # Malformed Joy.
        assert service(stop).success
    finally:
        executor.remove_node(gate)
        executor.remove_node(driver)
        gate.destroy_node()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
