#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from prius_msgs.msg import Control

import sys

controlTopic = "control" # Full name will be: <namespace>/<controlTopic>

BRAIN_FREQ = 20 # Hz

# States 
MOVE_FORWARD = 0
STOP = 1
TURN_LEFT = 2
TURN_RIGHT = 3
MOVE_BACKWARD = 4


sequence = [MOVE_FORWARD, STOP, TURN_LEFT, TURN_RIGHT, STOP, MOVE_BACKWARD, STOP]

class PriusMoveTest(Node):
    def __init__(self, node_name, name_space) -> None:
        super().__init__(node_name, namespace=name_space)
        qos = QoSReliabilityPolicy(1)
        
        # Create node variables    
        self.pub = self.create_publisher(Control, controlTopic, qos_profile=qos)
      
        self.iterationCount = 0
        self.action = 0
        self.timer = self.create_timer(1/BRAIN_FREQ, self.doWork)


    def doWork(self):
        
        # State change
        self.iterationCount += 1
        
        # State time is 5 sec
        if self.iterationCount == BRAIN_FREQ * 5:
            
            self.action += 1
            self.iterationCount = 0
            
        if self.action < len(sequence):
            action = sequence[self.action]
        else:
            print("Sequence ended...")
            action = STOP
            
        msg = Control()
        
        if action == MOVE_FORWARD:
            msg.throttle = 0.5
            msg.steer = 0.0
            msg.shift_gears = Control.FORWARD
            msg.brake = 0.0
            
        elif action == STOP:
            msg.throttle = 0.0
            msg.steer = 0.0
            msg.shift_gears = Control.NEUTRAL
            msg.brake = 1.0
            
        elif action == TURN_LEFT:
            msg.throttle = 0.5
            msg.steer = 1.0
            msg.shift_gears = Control.FORWARD
            msg.brake = 0.0
            
        elif action == TURN_RIGHT:
            msg.throttle = 0.5
            msg.steer = -1.0
            msg.shift_gears = Control.FORWARD
            msg.brake = 0.0
        
        elif action == MOVE_BACKWARD:
            msg.throttle = 0.5
            msg.steer = 0.0
            msg.shift_gears = Control.REVERSE
            msg.brake = 0.0
            
        self.pub.publish(msg)  

def main(args=None):
    rclpy.init(args=args)
    node = PriusMoveTest(node_name="prius_move_test", name_space="prius")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
