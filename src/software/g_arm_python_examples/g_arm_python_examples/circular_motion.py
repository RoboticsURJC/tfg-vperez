#!/usr/bin/env python3

from math import cos, sin

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2Servo
import g_arm_python_examples.g_arm_python_examples_lib.g_arm_pymoveit2 as g_arm


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 Servo interface
    moveit2_servo = MoveIt2Servo(
        node=node,
        frame_id=g_arm.base_link_name(),
        callback_group=callback_group
    )

    def servo_circular_motion():
        """Move in a circular motion using Servo"""

        now_sec = node.get_clock().now().nanoseconds * 1e-9
        
        linear=(sin(now_sec) / 20, cos(now_sec) / 20, 0.0)
        angular=(0.0, 0.0, 0.0)
        
        moveit2_servo(linear, angular)

    # Create timer for moving in a circular motion
    node.create_timer(0.2, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()