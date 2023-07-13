import rclpy
from rclpy.node import Node
from rclpy.action import *
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup 
from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import (
    JointTrajectoryPoint
)

class Controller(Node):

    def __init__(self):
        super().__init__('g_arm_controller')

        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        type(goal_request)
        return GoalResponse.ACCEPT

    async def execution(self, goal_handle):

        pass

    
    def cancel_callback(self, goal_handler):
        pass



    def timer_callback(self):
        msg = JointState()
        msg.name = ["joint1"]
        msg.position = [0.3]
        
        
        self.publisher_.publish(msg)
        self.i += 1




    def result_request_handler

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
