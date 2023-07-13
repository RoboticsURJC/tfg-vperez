import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import math
from g_arm.g_arm_lib import robot


class Driver(Node):
    
    _goal_joints_state = dict()
    
    def __init__(self):
        super().__init__('g_arm_driver')
        
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joints_state_callback, 10)
        
        self.robot = robot.Robot()
        
        self.get_logger().info("Connecting and calibrating the robot (May take a while)")
        
        if not self.robot.start():
            self.get_logger().error("Unable to start communications in /dev/ttyUSB0. Do you have permissions? (Try: sudo chmod 777 /dev/ttyUSB0)")
            exit(1)
            
        self.get_logger().info("Robot is ready to move!")
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.joints_state_apply)
           
    def joints_state_callback(self, msg):
        
        n_joints = len(msg.name)
        
        for i in range(n_joints):
            
            joint_name = msg.name[i]
            joint_position = msg.position[i]
            
            self._goal_joints_state[joint_name] = joint_position
            
        
    
    def joints_state_apply(self):
        
        j1 = math.degrees(self._goal_joints_state["joint1"])
        j2 = math.degrees(self._goal_joints_state["joint2"])
        j3 = math.degrees(self._goal_joints_state["joint3"])
        
        print([j1, j2, j3])
        self.robot.setAngles(j1, j2, j3)
    
def main(args=None):
    rclpy.init(args=args)

    driver = Driver()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
