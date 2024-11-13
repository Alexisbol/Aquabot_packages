from tutorial_interfaces.srv import Pathfind
from tutorial_interfaces.msg import Point
from tutorial_interfaces.msg import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import *
from transformations import euler_from_quaternion, quaternion_from_euler
import sys
from std_msgs.msg import Float64



def angle(p,c):
    return atan2(p[1]-c[1],p[0]-c[0])

def dist(a,b):
    return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))

def plusproche(pos,path):

    dm=dist(pos,path[0])
    im=0
    for i in range(len(path)-1) :
        d=dist(pos,path[i+1])
        if d<dm :
            dm=d
            im=i+1
    
    return path[max(len(path)-1,im+50)]

def tracking(pos,yaw,v,w,obj):
    tet=(angle(obj,pos)-yaw)%(2*pi)
    somme=max(cos(tet),0)
    diff=sin(tet)
    if tet>pi/2:diff=1
    if tet<-pi/2:diff=-1
    rt=(somme+diff)*50000
    lt=(somme-diff)*50000
    return (rt,lt,tet,somme,diff,yaw)


class Tracking(Node):

    def __init__(self):
        super().__init__('tracking')
        self.path=[]
        self.posbateau=(0,0)
        self.vbateau=(0,0)
        
        self.roll, self.pitch, self.yaw = 0,0,0
        self.wbateau=(0,0)
        self.cli = self.create_client(Pathfind, 'FindPath')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Pathfind.Request()
        
        # Créer un abonnement au topic "aquabot/odom"
        # Le type de message est nav_msgs/msg/Odometry

        self.subscription = self.create_subscription(
            Odometry,
            'aquabot/odom',
            self.odom_callback,
            10  # QoS profile (10 est une bonne valeur par défaut)
        )

        
        self.publisherl = self.create_publisher(Float64,'/aquabot/thrusters/left/thrust',10)
        self.timer=self.create_timer(0.5,self.commandel_callback)
        self.publisherr = self.create_publisher(Float64,'/aquabot/thrusters/right/thrust',10)
        self.timer=self.create_timer(0.5,self.commander_callback)
        
        self.odom_received = False
        self.path_received = False

    def send_request(self, start, goal):
        self.req.start.x = start[0]
        self.req.start.y = start[1]
        self.req.goal.x = goal[0]
        self.req.goal.y = goal[1]
        future = self.cli.call_async(self.req)
        def handle_response(future):
            response = future.result()
            path = [(point.x, point.y) for point in response.path.points]
            self.path_received=True
            self.get_logger().info(f"Received path")
            self.path = path  # Enregistrer le chemin dans l'attribut

        future.add_done_callback(handle_response)
        return future
    
    def odom_callback(self, msg):
        self.posbateau=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.vbateau=(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        
        #self.yaw, self.pitch, self.roll = euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.yaw=2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.wbateau=(msg.twist.twist.angular.z)
        self.odom_received = True
        # Afficher les informations de position et de vitesse reçues
        self.get_logger().info(f"Position -> x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}, z: {msg.pose.pose.position.z}")
        self.get_logger().info(f"Orientation -> x: {msg.pose.pose.orientation.x}, y: {msg.pose.pose.orientation.y}, z: {msg.pose.pose.orientation.z}, w: {msg.pose.pose.orientation.w}")
        self.get_logger().info(f"Orientation -> yaw: {self.yaw}")#, roll: {self.roll}, pitch: {self.pitch}")

        self.get_logger().info(f"Vitesse linéaire -> x: {msg.twist.twist.linear.x}, y: {msg.twist.twist.linear.y}, z: {msg.twist.twist.linear.z}")
        self.get_logger().info(f"Vitesse angulaire -> x: {msg.twist.twist.angular.x}, y: {msg.twist.twist.angular.y}, z: {msg.twist.twist.angular.z}")


    
    def commandel_callback(self):
        if self.odom_received and self.path:
            msg=Float64()
            #self.get_logger().info('Path: "%s"' % self.path)
            (rt,lt,tet,somme,diff,yaw)= tracking(self.posbateau,self.yaw,self.vbateau,self.wbateau,plusproche(self.posbateau,self.path))
            self.get_logger().info('accel gauche: "%s"' % lt)

            msg.data=float(lt)
            self.publisherl.publish(msg)

    def commander_callback(self):
        if self.odom_received and self.path:
            msg=Float64()
            self.get_logger().info('self.yaw: "%s"' % self.yaw)

            (rt,lt,tet,somme,diff,yaw)= tracking(self.posbateau,self.yaw,self.vbateau,self.wbateau,plusproche(self.posbateau,self.path))
            msg.data=float(rt)
            self.publisherr.publish(msg)
            self.get_logger().info('accel droit: "%s"' % msg.data)
            self.get_logger().info('tet: "%s"' % tet)
            self.get_logger().info('yaw: "%s"' % yaw)

            self.get_logger().info('diff: "%s"' % diff)
            self.get_logger().info('somme: "%s"' % somme)





def main():
    rclpy.init()

    # Créer le nœud d'écoute
    tracking = Tracking()
    
    
            
    future = tracking.send_request((float(sys.argv[1]), float(sys.argv[2])),(float(sys.argv[3]),float(sys.argv[4])))
    rclpy.spin_until_future_complete(tracking, future)
    
    if tracking.path:
        print(f"Path received: {tracking.path}")
    else:
        tracking.get_logger().error("No path received")

    rclpy.spin(tracking)

        
    # Détruire le nœud proprement avant de quitter
    tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
