import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry

def yaw_from_quaternion(quaternion):
    z = quaternion.z
    w = quaternion.w
    return 2 * np.arctan2(z, w)

class CameraControl(Node):
    def __init__(self):
        super().__init__('camera_control')
        self.camera_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 5)
        self.subscription_odom = self.create_subscription(Odometry, '/aquabot/odom', self.odom_callback, 10)
        self.subscription_aim = self.create_subscription(Point, '/aquabot/camera_look_at', self.aim_callback, 10)
        
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

def main(args=None):
    rclpy.init(args=args)
    camera_turn_node = CameraControl()
    
    # Ajout d'un timer pour la mise à jour périodique
    timer_period = 0.05  # 50ms
    camera_turn_node.create_timer(timer_period, camera_turn_node.timer_callback)
    
    rclpy.spin(camera_turn_node)
    camera_turn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()