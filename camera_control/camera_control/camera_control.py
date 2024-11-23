import rclpy
import math
#import numpy as np
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np

def yaw_from_quaternion(quaternion):
    #x = quaternion.x
    #y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    #siny_cosp = 2 * (w * z + x * y)
    #cosy_cosp = 1 - 2 * (y * y + z * z)
    #yaw = np.arctan2(siny_cosp, cosy_cosp)
    yaw = 2*np.arctan2(z,w)
    return yaw


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
            30)
            
        self.subscription = self.create_subscription(
            Point,
            '/aquabot/camera_look_at',
            self.aim_callback,
            10)
        
        self.odom_pose = None
        self.aim_point = None

    def calculate_optimal_angle(self, current_yaw, target_angle):
        # Calcul du chemin le plus court entre deux angles
        angle_diff = (target_angle - current_yaw + np.pi) % (2 * np.pi) - np.pi
        return angle_diff

    def timer_callback(self):
        if self.odom_pose is not None and self.aim_point is not None:
            odom_point = self.odom_pose.position
            odom_quaternion = self.odom_pose.orientation
            current_yaw = yaw_from_quaternion(odom_quaternion)

            # Calculer l'angle vers le point de visée
            target_angle = np.arctan2(self.aim_point.y - odom_point.y, 
                                      self.aim_point.x - odom_point.x)
            
            # Ajuster l'angle pour le moteur de la caméra
            camera_angle = self.calculate_optimal_angle(current_yaw, target_angle)
            
            camera_turn_msg = Float64()
            camera_turn_msg.data = camera_angle
            self.camera_turn_pub.publish(camera_turn_msg)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        
        
    def aim_callback(self, msg):
        self.aim_point = msg
        #self.get_logger().info('Received point to look at')
	

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
