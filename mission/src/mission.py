import rclpy
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from math import *

class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/aquabot/odom',
            self.odom_callback,
            10)
        
        self.turbines_subscription = self.create_subscription(
            PoseArray,
            '/aquabot/turbines',
            self.turbinespose_callback,
            10
        )

        self.phase_subscription = self.create_subscription(
            UInt32,
            '/vrx/windturbineinspection/current_phase',
            self.phase_callback,
            10
        )

        self.qr_subscription = self.create_subscription(
            String,
            '/aquabot/qrcode_data',
            self.qrcode_callback,
            10
        )

        self.goal_publishers = self.create_publisher(Point,'/aquabot/goal',10)
        self.camera_publishers = self.create_publisher(Point,'/aquabot/camera_look_at',10)
        self.qr_publishers = self.create_publisher(String,'/vrx/windturbineinspection/windturbine_checkup',10)

        self.timers = self.create_timer(0.1, self.timer_callback)

        self.odom = Odometry()
        self.odom_received = False
        self.liste_turbines = PoseArray()
        self.turbines_received = False
        self.qrcode = String()
        self.qrcode_received = False
        self.phase = UInt32

        self.turbinesI = 0
        self.currentgoal = Point()

        self.status = 'INITIALIZED'

    def proche_goal(self,dist):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        gx = self.currentgoal.x
        gy = self.currentgoal.y
        if(abs(x-gx)<dist and abs(y-gy)<dist):
            return True
        else:
            return False
        
    

    def point_tangeant(position,objectif):
        disteol=10

        xo,yo=objectif
        x,y=position
        gamma=atan2(yo-y,xo-x)
        tetha=acos(disteol/dist(position,objectif))
        point=(xo+disteol*cos(gamma+tetha-pi),yo+disteol*sin(gamma+tetha-pi))
        return point

    def odom_callback(self,msg):
        self.odom = msg.data
        self.odom_received = True

    def turbinespose_callback(self,msg):
        self.liste_turbines = msg.data
        self.turbines_received = True

    def phase_callback(self,msg):
        self.phase = msg.data

    def qrcode_callback(self,msg):
        if(msg.data != None):
            self.qrcode = msg.data
            self.qrcode_received = True

    def timer_callback(self):
        if(self.status == 'INITIALIZED'):
            if(self.turbines_received and self.phase == 1):
                self.status = 'SEARCH'

        if(self.status == 'SEARCH'):
            #a modif mettre un point proche mais pas exacte
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            self.currentgoal = self.point_tangeant((x,y),self.liste_turbines[self.turbinesI])

            self.goal_publishers.publish(self.currentgoal)
            self.camera_publishers.publish(self.liste_turbines[self.turbinesI])
            
            if(not self.proche_goal(20)): #pas assez proche pour etre sur que ce soit le bon qrcode
                self.qrcode_received = False

            if(self.qrcode_received): #QR code scanné
                self.turbinesI+=1
                self.qr_publishers.publish(self.qrcode)
            elif(self.proche_goal(1)): #Arrivé mais pas QR code scanné
                #mettre le point en face du point actuel pour forcer à faire le tour
                self.currentgoal = self.liste_turbines[self.turbinesI]

            if(self.phase == 2):
                self.status = 'RALLY'


            






def main(args=None):
    rclpy.init(args=args)

    Mission = Mission()

    rclpy.spin(Mission)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
