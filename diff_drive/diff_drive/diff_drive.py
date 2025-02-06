import rclpy
import math
#import numpy as np
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np


class DiffDriveControl(Node):

    def __init__(self):
        super().__init__('DiffDriveControl')

        self.publisher_l = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust',10)
        self.publisher_r = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust',10)
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            30)
        
        self.cmd_vel_x = None
        self.cmd_vel_rotation_z = None

    def timer_callback(self):
        if self.cmd_vel_x is not None and self.cmd_vel_rotation_z is not None:
            v = self.cmd_vel_x
            omega = np.cos(self.cmd_vel_rotation_z)
            l = 0.2
            r = 0.1
            left_thrust = (v - (l/2)*omega)/r
            right_thrust = (v + (l/2)*omega)/r
            msg_l = Float64()
            msg_l.data = left_thrust
            msg_r = Float64()
            msg_r.data = right_thrust
            self.publisher_l.publish(msg_l)
            self.publisher_r.publish(msg_r)
            self.get_logger().info('Publishing left thrust: "%f"' % msg_l.data)
            self.get_logger().info('Publishing right thrust: "%f"' % msg_r.data)


    def cmd_vel_callback(self, msg):
        self.cmd_vel_x = msg.linear.x
        self.cmd_vel_rotation_z = msg.angular.z
        
	

def main(args=None):
    rclpy.init(args=args)

    diff_drive_pubsub = DiffDriveControl()

    rclpy.spin(diff_drive_pubsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    diff_drive_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
