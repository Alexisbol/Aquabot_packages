import rclpy
import math
#import numpy as np
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *



class CameraControl(Node):

    def __init__(self):
        super().__init__('camera_control')

        self.camera_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 5)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            Odometry,
            '/aquabot/odom',
            self.odom_callback,
            10)
            
        self.subscription = self.create_subscription(
            Point,
            '/aquabot/camera_look_at',
            self.aim_callback,
            10)
        
        self.odom_pose = None
        self.aim_point = None

        
    def timer_callback(self):
        #self.get_logger().info('udpate_odom: "%s"' % self.odom_pose)
        #self.get_logger().info('udpate_aim: "%s"' % self.aim_point)
        if self.odom_pose != None and self.aim_point != None:
            camera_turn_msg = Float64()
        
            odom_point = self.odom_pose.position
            odom_angle = self.odom_pose.orientation.z
            self.get_logger().info('odomangle: "%s"' % odom_angle)
            angle1 = math.atan((self.aim_point.x - odom_point.x)/(self.aim_point.y - odom_point.y))
            self.get_logger().info('angle1: "%s"' % angle1)
            camera_turn_msg.data = (angle1 - odom_angle)

            self.get_logger().info('Publishing: "%s"' % camera_turn_msg.data)
            self.camera_turn_pub.publish(camera_turn_msg)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        
        
    def aim_callback(self, msg):
        self.aim_point = msg
        self.get_logger().info('Received point to look at')
	

def main(args=None):
    rclpy.init(args=args)

    camera_turn_pubsub = CameraControl()

    rclpy.spin(camera_turn_pubsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_turn_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()