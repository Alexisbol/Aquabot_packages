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
from std_msgs.msg import Float64, UInt32
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

def commande(pos, theta, v_actual, omega_actual, objectif):
    #Fonction qui détermine la commande à envoyer à nos 2 moteurs pour suivre l'objectif
    x_target, y_target=objectif
    x,y=pos
    # Calcul de la distance et de l'angle vers la cible
    distance_target = sqrt((x_target - x)**2 + (y_target - y)**2)
    theta_target = atan2(y_target - y, x_target - x)
    
    # Calcul de l'erreur d'angle
    angle_error = theta_target - theta
    norme_angle_error = sqrt(angle_error**2)
    # Normaliser l'angle dans l'intervalle [-pi, pi]
    angle_error = atan2(sin(angle_error),cos(angle_error))
    
    # Définir la vitesse linéaire et angulaire désirée
    v_desired = k_d * distance_target # ((1/(0.005+sqrt(angle_error**2)))+ distance_target)
    omega_desired = k_theta * angle_error
    
    # Calcul des erreurs de vitesse linéaire et angulaire
    delta_v = v_desired - norme(v_actual)
    norme_delta_v = sqrt(delta_v**2)
    delta_omega = omega_desired - omega_actual
    
    # Calculer les commandes pour chaque moteur en compensant les erreurs
    V_gauche = k_v * delta_v + k_omega * delta_omega
    V_droite = k_v * delta_v - k_omega * delta_omega
    
    return V_gauche, V_droite,angle_error,delta_v,delta_omega,theta

class Tracking(Node):
    def __init__(self):
        super().__init__('tracking')
        self.path=[]
        self.pathfixed = Pathfixed()
        self.posbateau=(0,0)
        self.vbateau=(0,0)
        self.goal = Point()
        self.commande_type = UInt32()
        self.commande_type.data = 1
        
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
        
        self.phase_subscription = self.create_subscription(
            UInt32,
            '/vrx/windturbinesinspection/current_phase',
            self.phase_callback,
            10
        )
        
        self.commande_type_subscription = self.create_subscription(
            UInt32,
            '/aquabot/commande_type',
            self.commande_type_callback,
            10
        )

        self.commande_type_subscription = self.create_subscription(
            Float64,
            '/aquabot/qrcode_angle',
            self.qrcode_angle_callback,
            10
        )

        self.commande_type_subscription = self.create_subscription(
            Float64,
            '/aquabot/flitered_distance',
            self.flitered_distance_callback,
            10
        )

        self.commande_type_subscription = self.create_subscription(
            Float64,
            '/aquabot/flitered_angle',
            self.flitered_angle_callback,
            10
        )

        self.publisherl = self.create_publisher(Float64,'/aquabot/thrusters/left/thrust',10)
        self.timer=self.create_timer(0.5,self.commandel_callback)
        self.publisherr = self.create_publisher(Float64,'/aquabot/thrusters/right/thrust',10)
        self.timer=self.create_timer(0.5,self.commander_callback)
        self.timer=self.create_timer(1,self.updatepath_callback)
        self.publisherpath = self.create_publisher(Pathfixed,'/aquabot/currentpath',10)

        self.publisher_pos_l = self.create_publisher(Float64,'/aquabot/thrusters/left/pos',10)
        self.publisher_pos_r = self.create_publisher(Float64,'/aquabot/thrusters/right/pos',10)
        self.timer=self.create_timer(0.5,self.stabiliser_position)


        self.qr_angle = Float64()
        self.qr_angle.data = 10.0

        self.flitered_distance = Float64()
        self.flitered_distance.data = 0.0

        self.flitered_angle = Float64()
        self.flitered_angle.data = 0.0

        self.odom_received = False
        self.goal_received = False
        self.qrcode_angle_received = True

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
    
    def commande_type_callback(self,msg):
        self.commande_type = msg

    def flitered_distance_callback(self,msg):
        self.flitered_distance = msg

    def flitered_angle_callback(self,msg):
        self.flitered_angle = msg
    
    def qrcode_angle_callback(self,msg):
        self.qr_angle = msg
        self.qrcode_angle_received = True

    def phase_callback(self,msg):
        self.phase = msg.data

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
    
    def commandel_callback(self):
        if self.odom_received and self.path and self.commande_type.data==1:
            msg=Float64()
            #self.get_logger().info('Path: "%s"' % self.path)
            (right_Thrust,lt,theta,somme,diff,yaw)= commande(self.posbateau,self.yaw,self.vbateau,self.wbateau,plusproche(self.posbateau,self.path))
            #self.get_logger().info('angle_error: "%s"' % theta)

            msg.data=float(lt)
            self.publisherl.publish(msg)

    def commander_callback(self):
        if self.odom_received and self.path and self.commande_type.data==1:
            msg=Float64()
            #self.get_logger().info('self.yaw: "%s"' % self.yaw)
            self.objectif=plusproche(self.posbateau,self.path)
            (right_Thrust,lt,angle_error,somme,diff,yaw)= commande(self.posbateau,self.yaw,self.vbateau,self.wbateau,self.objectif)                        
            
            msg.data=float(right_Thrust)
            self.publisherr.publish(msg)

            #self.get_logger().info('accel droit: "%s"' % msg.data)
            #self.get_logger().info(f"objectif :{self.objectif}" ) 
            #self.get_logger().info(f"pos:{self.posbateau}")
            self.objectif=plusproche(self.posbateau,self.path)

            #self.get_logger().info(f"angle:{angle(self.objectif,self.posbateau)}")
            #self.get_logger().info('angle_error: "%s"' % angle_error)
            #self.get_logger().info('yaw: "%s"' % yaw)

            #self.get_logger().info('deltav: "%s"' % diff)
            #self.get_logger().info('deltaomega: "%s"' % somme)

    def stabiliser_position(self):
        if self.commande_type.data == 2:
            # Récupération des données filtrées
            err_angle = self.filtered_angle.data
            err_angle = atan2(sin(err_angle), cos(err_angle))

            distance_reelle = self.filtered_distance.data

            # Paramètres
            distance_voulue = 10.0  # Distance cible
            MAX_THRUST = 5000.0
            K_p_distance = 10 # Gain proportionnel pour la distance
            K_p_angle = 9  # Gain proportionnel pour l'angle

            # Ajustement de l'orientation
            msg_pos_left = Float64()
            msg_pos_right = Float64()

            if abs(err_angle) > 0.05:  # Si l'erreur d'angle est significative
                motor_angle = max(min(err_angle, pi / 4), -pi / 4)  # Limitation entre -π/4 et π/4
                msg_pos_left.data = motor_angle
                msg_pos_right.data = -motor_angle  # Inversion pour tourner
            else:
                msg_pos_left.data = 0.0
                msg_pos_right.data = 0.0

            # Publication des ajustements moteurs
            self.publisher_pos_l.publish(msg_pos_left)
            self.publisher_pos_r.publish(msg_pos_right)

            # Calcul de la poussée
            distance_error = distance_reelle - distance_voulue
            base_thrust = K_p_distance * distance_error
            angle_correction = K_p_angle * err_angle

            thrust_left = Float64()
            thrust_right = Float64()

            # Calcul de la poussée totale
            thrust_left.data = max(min(base_thrust + angle_correction, MAX_THRUST), -MAX_THRUST)
            thrust_right.data = max(min(base_thrust - angle_correction, MAX_THRUST), -MAX_THRUST)

            # Réduction de la poussée si proche de la cible
            if abs(distance_error) < 3:
                thrust_left.data *= 0.5
                thrust_right.data *= 0.5

            # Publication de la poussée
            self.publisherl.publish(thrust_left)
            self.publisherr.publish(thrust_right)

            # Logs pour le débogage
            self.get_logger().info(f"pos_left: {msg_pos_left.data}")
            self.get_logger().info(f"pos_right: {msg_pos_right.data}")
            self.get_logger().info(f"thrust_left: {thrust_left.data}")
            self.get_logger().info(f"thrust_right: {thrust_right.data}")


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
