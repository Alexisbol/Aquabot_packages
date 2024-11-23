from custom_interface.srv import Pathfind
from custom_interface.msg import Point
from custom_interface.msg import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as Pathfixed
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point as Pointfixed
from math import *
#from transformations import euler_from_quaternion, quaternion_from_euler
import sys
from std_msgs.msg import Float64
import numpy as np
from sensor_msgs.msg import Imu

from ros_gz_interfaces.msg import ParamVec


def euler_from_quaternion(x,y,z,w):
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def angle(p,c):
    return atan2(p[1]-c[1],p[0]-c[0])

def dist(a,b):
    return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))

def plusproche(pos,path):
    dist_min=dist(pos,path[0])
    im=0
    for i in range(len(path)-1) :
        d=dist(pos,path[i+1])
        if d<dist_min :
            dist_min=d
            im=i+1
    return path[min(len(path)-1,im+10)]

def norme(v):
    return dist(v,(0,0))

# Constantes de proportionnalité
k_d = 12    # Constante pour la distance
k_theta = 13  # Constante pour l'orientation
k_v = 4   # Constante pour ajuster la puissance du moteur linéaire
k_omega = 40  # Constante pour ajuster la puissance du moteur angulaire

Kp = 5
Ktheta = 10

def commande(pos, theta, objectif, objectiforientation = "None"):
    #Fonction qui détermine la commande à envoyer à nos 2 moteurs pour suivre l'objectif
    x_target, y_target=objectif
    x,y=pos

    if(objectiforientation == "None"):
        vect = (x_target - x,y_target - y)
        norm = np.sqrt(vect[0]**2 + vect[1]**2)
        vect = (5*vect.x/norm,5*vect.y/norm)
        objectiforientation = (x_target+vect[0],y_target+vect[1])

    deltaX = x_target-x
    deltaY = y_target-y
    norm = np.sqrt(deltaX**2 + deltaY**2)

    alpha = np.arctan2(deltaY, deltaX) - theta

    if(abs(alpha)<np.pi/4):
        phi = alpha
        Np = Kp*norm
    else:
        phi = alpha + np.pi
        Np = -1*Kp*norm

    deltaTheta = np.arctan2(objectiforientation[1]-y,objectiforientation[0]-x)-theta
    Ntheta = Ktheta*deltaTheta

    Nd = Np + Ntheta
    Ng = Np - Ntheta
    return Nd,Ng,phi

class Tracking(Node):
    def __init__(self):
        super().__init__('tracking')
        self.path=[]
        self.pathfixed = Pathfixed()
        self.posbateau=(0,0)
        self.vbateau=(0,0)
        self.goal = Point()
        
        self.roll, self.pitch, self.yaw = 0,0,0
        self.wbateau=(0,0)
        self.cli = self.create_client(Pathfind, 'FindPath')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Pathfind.Request()
        
        self.subscription = self.create_subscription(
            Pointfixed,
            'aquabot/goal',
            self.goal_callback,
            10
        )

        # Créer un abonnement au topic "aquabot/odom"
        # Le type de message est nav_msgs/msg/Odometry
        self.subscription = self.create_subscription(
            Odometry,
            'aquabot/odom',
            self.odom_callback,
            10  # QoS profile (10 est une bonne valeur par défaut)
        )

        self.subscription = self.create_subscription(
            Imu,
            'aquabot/imu',
            self.imu_callback,
            10  # QoS profile (10 est une bonne valeur par défaut)
        )

        self.ping_subscription = self.create_subscription(
            ParamVec,
            'aquabot/sensors/acoustics/receiver/range_bearing',
            self.ping_callback,
            10)
        
        self.publisherl = self.create_publisher(Float64,'/aquabot/thrusters/left/thrust',10)
        self.timer=self.create_timer(0.2,self.commande_callback)
        self.publisherr = self.create_publisher(Float64,'/aquabot/thrusters/right/thrust',10)
        self.timer=self.create_timer(1,self.updatepath_callback)
        self.publisherpath = self.create_publisher(Pathfixed,'/aquabot/currentpath',10)

        self.publisher_pos_l = self.create_publisher(Float64,'/aquabot/thrusters/left/pos',10)
        self.publisher_pos_r = self.create_publisher(Float64,'/aquabot/thrusters/right/pos',10)

        self.odom_received = False
        self.goal_received = False

    def send_request(self, start, goal):
        self.req.start.x = start[0]
        self.req.start.y = start[1]
        self.req.goal.x = goal[0]
        self.req.goal.y = goal[1]
        future = self.cli.call_async(self.req)
        def handle_response(future):
            response = future.result()
            path = [(point.x, point.y) for point in response.path.points]
            pathfixed = Pathfixed()
            # Add header to each pose
            pathfixed.header.frame_id = "world"
            pathfixed.header.stamp = self.get_clock().now().to_msg()
            for point in response.path.points:
                posefixed = PoseStamped()
                # Add header to each pose
                posefixed.header.frame_id = "world"
                posefixed.header.stamp = self.get_clock().now().to_msg()
            
                posefixed.pose.position.x = point.x
                posefixed.pose.position.y = point.y
                posefixed.pose.position.z = 0.0
                pathfixed.poses.append(posefixed)
            self.path_received=True
            #self.get_logger().info(f"Received path")
            self.path = path  # Enregistrer le chemin dans l'attribut
            self.pathfixed = pathfixed
            #self.get_logger().info(path)
        future.add_done_callback(handle_response)
        return future
    
    def goal_callback(self,msg):
        self.goal = (msg.x,msg.y)
        self.goal_received = True

    def updatepath_callback(self):
        if(self.odom_received and self.goal_received):
            self.send_request(self.posbateau,self.goal)
            self.publisherpath.publish(self.pathfixed)
    
    def odom_callback(self, msg):
        self.posbateau=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.vbateau=(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        #self.yaw=2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.wbateau=(msg.twist.twist.angular.z)
        self.odom_received = True
        # Afficher les informations de position et de vitesse reçues
        #self.get_logger().info(f"Position -> x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}, z: {msg.pose.pose.position.z}")
        #self.get_logger().info(f"Orientation -> x: {msg.pose.pose.orientation.x}, y: {msg.pose.pose.orientation.y}, z: {msg.pose.pose.orientation.z}, w: {msg.pose.pose.orientation.w}")
        #self.get_logger().info(f"Orientation -> yaw: {self.yaw}, roll: {self.roll}, pitch: {self.pitch}")

        #self.get_logger().info(f"Vitesse linéaire -> x: {msg.twist.twist.linear.x}, y: {msg.twist.twist.linear.y}, z: {msg.twist.twist.linear.z}")
        #self.get_logger().info(f"Vitesse angulaire -> x: {msg.twist.twist.angular.x}, y: {msg.twist.twist.angular.y}, z: {msg.twist.twist.angular.z}")

    def imu_callback(self, msg: Imu):
        # Extraire les quaternions
        orientation_x = msg.orientation.x
        orientation_y = msg.orientation.y
        orientation_z = msg.orientation.z
        orientation_w = msg.orientation.w
        
        # Afficher les quaternions dans le log
        #self.get_logger().info(f"Orientation Quaternion -> x: {orientation_x}, y: {orientation_y}, z: {orientation_z}, w: {orientation_w}")
        #self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z, msg.orientation.w)
        #self.get_logger().info(f"Orientation -> yaw: {self.yaw}, roll: {self.roll}, pitch: {self.pitch}")

    def ping_callback(self,msg):
        a = 0
    
    def commande_callback(self):
        if self.odom_received and self.path:
            Ndmsg=Float64()
            Ngmsg=Float64()
            Thetamsg=Float64()
            (Nd,Ng,theta)= commande(self.posbateau,self.yaw,plusproche(self.posbateau,self.path))

            Ndmsg.data=float(Nd)
            Ngmsg.data=float(Ng)
            Thetamsg.data=float(theta)

            self.publisherl.publish(Ngmsg)
            self.publisherr.publish(Ndmsg)
            self.publisher_pos_l.publish(Thetamsg)
            self.publisher_pos_r.publish(Thetamsg)

def main():
    rclpy.init()

    # Créer le nœud d'écoute
    tracking = Tracking()
            
    #future = tracking.send_request((float(sys.argv[1]), float(sys.argv[2])),(float(sys.argv[3]),float(sys.argv[4])))
    #rclpy.spin_until_future_complete(tracking, future)
    #
    #if tracking.path:
    #    print(f"Path received: {tracking.path}")
    #else:
    #    tracking.get_logger().info("No path received")

    rclpy.spin(tracking)

        
    # Détruire le nœud proprement avant de quitter
    tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
