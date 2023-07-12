import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

class Controller(Node):

    def __init__(self):
        super().__init__('g_arm_controller')
        
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ["joint1"]
        msg.position = [0.3]
        
        
        self.publisher_.publish(msg)
        self.i += 1


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
