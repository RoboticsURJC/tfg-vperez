import rclpy
from rclpy.node import Node

import robot

class RobotNode(Node):

    def __init__(self):
        super().__init__('g_arm')

        self.__rob = robot.Robot()
        
        if not self.__rob.start():
            exit(1)

             
    # Private
    __rob = None

def main(args=None):
    rclpy.init(args=args)

    g_arm = RobotNode()

    rclpy.spin(g_arm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()