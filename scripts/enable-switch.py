

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy



class EnablePublisher(Node):

    def __init__(self):
        super().__init__('enabler')
        self.publisher_ = self.create_publisher(Bool, 'enable', 0)
        self.proceed_publisher_ = self.create_publisher(Bool, 'proceed', 0)

        self.gripper_open_publisher_ = self.create_publisher(Bool, 'gripper_open', 0)
        self.gripper_close_publisher_ = self.create_publisher(Bool, 'gripper_close', 0)

        self.i = 0
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            0)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        pressed = msg.buttons[10]
        proceed = msg.buttons[0] + msg.buttons[1]

        gripper_open = (True if msg.axes[6] > 0 else False)
        gripper_close = (True if msg.axes[6] < 0 else False)

        msg = Bool()
        msg.data = (True if pressed == 1 else False)
        self.publisher_.publish(msg)
        msg.data = (True if proceed > 0.1 else False)
        self.proceed_publisher_.publish(msg)

        msg.data = (True if gripper_open == 1 else False)
        self.gripper_open_publisher_.publish(msg)
        msg.data = (True if gripper_close == 1 else False)
        self.gripper_close_publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    enabler_publisher = EnablePublisher()

    rclpy.spin(enabler_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    enabler_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
