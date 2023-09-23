#!/usr/bin/env python3

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
import g_arm_python_examples.g_arm_python_examples_lib.g_arm_pymoveit2 as g_arm
from g_arm_python_examples.drawFigures import *

class Trajectory2D:

    def __init__(self):

        rclpy.init()

        # Create node for this example
        self._node = Node("ex_trajectory")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self._moveit2 = MoveIt2(
            node=self._node,
            joint_names=g_arm.joint_names(),
            base_link_name=g_arm.base_link_name(),
            end_effector_name=g_arm.end_effector_name(),
            group_name=g_arm.MOVE_GROUP_ARM,
            callback_group=callback_group,
            follow_joint_trajectory_action_name="/arm_controller/follow_joint_trajectory",
            execute_via_moveit=False
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self._node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

    def __cartesianSpaceTojointsSpace(self, points):
        
        jointsStates = []
        for p in points:
            x = p[0]
            y = p[1]
            jointStateObj = self._moveit2.compute_ik((x, y, 0.0), (1.0, 0.0, 0.0, 0.0))
            
            j1j2j3 = (jointStateObj.position[0], jointStateObj.position[1], jointStateObj.position[2])
            jointsStates.append(j1j2j3)

        return jointsStates
    
    def follow(self, points2D):
        
        for p in points2D:
            position = (p[0], p[1], 0.15)
            print("Actual: " + str(position))
            self._moveit2.move_to_pose(position=position, quat_xyzw=(1.0, 0.0, 0.0, 0.0), cartesian=True, 
                         frame_id=g_arm.base_link_name(), target_link=g_arm.end_effector_name(), tolerance_orientation=3.14)
            self._moveit2.wait_until_executed()
        
        rclpy.shutdown()
        exit(0)

def main():
    traj = Trajectory2D()
    #verticesSquare((0.25, 0.0), 0.12)
    points = verticesHeart((0.25, 0.0), 0.005)
    traj.follow(points)

if __name__ == "__main__":
    main()
