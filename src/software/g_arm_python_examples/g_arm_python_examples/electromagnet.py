#!/usr/bin/env python3

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
from pymoveit2 import MoveIt2Gripper
import g_arm_python_examples.g_arm_python_examples_lib.g_arm_pymoveit2 as g_arm
from g_arm_python_examples.drawFigures import *

class ElectromagnetExample:

    def __init__(self):

        rclpy.init()

        # Create node for this example
        self._node = Node("ex_electromagnet")     

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
        
        # Create MoveIt 2 gripper interface
        self._tool = MoveIt2Gripper(
        node=self._node,
        gripper_joint_names=g_arm.tool_joint_name(),
        open_gripper_joint_positions=g_arm.electromagnet_on(),
        closed_gripper_joint_positions=g_arm.electromagnet_off(),
        gripper_group_name=g_arm.MOVE_GROUP_GRIPPER,
        follow_joint_trajectory_action_name="/tool_controller/follow_joint_trajectory",
        callback_group=callback_group,
        )


        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self._node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
    
    def moveTo(self, position):
            self._moveit2.move_to_pose(position=position, quat_xyzw=[1.0, 0.0, 0.0, 0.0], cartesian=True, 
                         frame_id=g_arm.base_link_name(), target_link=g_arm.end_effector_name(), tolerance_orientation=100.0)
            self._moveit2.wait_until_executed()
    def magnetOn(self):
        self._tool.open()
        self._tool.wait_until_executed()
    
    def magnetOff(self):
        self._tool.close()
        self._tool.wait_until_executed()
    
    def behaviour(self):
        
        self.magnetOff()
        self.moveTo((0.2, 0.2, 0.1))
        
        self.moveTo((0.2, 0.2, 0.05))
        self.magnetOn()
        self.moveTo((0.2, 0.2, 0.1))
        
        self.moveTo((0.3, 0.0, 0.1))
        self.moveTo((0.3, 0.0, 0.05))
        self.magnetOff()
        self.moveTo((0.3, 0.0, 0.1))
        
        self.moveTo((0.3, 0.0, 0.2))
        
        rclpy.shutdown()
        exit(0)

def main():
    
    example = ElectromagnetExample()
    
    example.behaviour()

if __name__ == "__main__":
    main()
