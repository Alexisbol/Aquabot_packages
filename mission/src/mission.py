#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from ros_gz_interfaces.msg import ParamVec

class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        self.ping_subscription = self.create_subscription(
            ParamVec,
            'aquabot/sensors/acoustics/receiver/range_bearing',
            self.ping_callback(),
            10)

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

        self.timer = self.create_timer(0.5, self.timer_callback)

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

    def odom_callback(self,msg):
        self.odom = msg
        self.odom_received = True

    def ping_callback(self,msg):
        self.ping = msg
        self.ping_received = True

    def turbinespose_callback(self,msg):
        self.liste_turbines = msg
        self.turbines_received = True

    def phase_callback(self,msg):
        self.phase = msg

    def qrcode_callback(self,msg):
        if(msg.data != None):
            self.qrcode = msg.data
            self.qrcode_received = True

    def timer_callback(self):
        self.get_logger().info(self.status)

        if(self.status == 'INITIALIZED'):
            if(self.turbines_received):
                self.status = 'SEARCH'

        if(self.status == 'SEARCH'):
            #a modif mettre un point proche mais pas exacte
            self.currentgoal = self.liste_turbines[self.turbinesI]

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

        if(self.status == 'RALLY'):
            self.turbinesI = 0
            self.currentgoal = self.liste_turbines[self.turbinesI] #ca faut voir avec sirena pour l'id defectueuse

            #if(self.proche_goal(15)):
                #utiliser la commande de stabilisation
                

            




            






def main(args=None):
    rclpy.init(args=args)

    mission = Mission()

    rclpy.spin(mission)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
