#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from ros_gz_interfaces.msg import ParamVec
import numpy as np

class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        self.ping_subscription = self.create_subscription(
            ParamVec,
            'aquabot/sensors/acoustics/receiver/range_bearing',
            self.ping_callback,
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
            '/vrx/windturbinesinspection/current_phase',
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
        self.qr_publishers = self.create_publisher(String,'/vrx/windturbinesinspection/windturbine_checkup',10)

        self.timer = self.create_timer(1, self.timer_callback)

        self.odom = Odometry()
        self.odom_received = False
        self.liste_turbines = PoseArray()
        self.turbines_received = False
        self.qrcode = String()
        self.qrcode_received = False
        self.phase = UInt32

        self.turbinesI = 0
        self.currentgoal = Point()
        self.currentcameragoal = Point()
        self.point = Point()
        self.liste_turbines_reste = PoseArray()

        self.status = 'INITIALIZED'

    def proche_goal(self,dist):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        gx = self.currentgoal.position.x
        gy = self.currentgoal.position.y
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
        self.liste_turbines = msg.poses
        if(not self.turbines_received):
            self.liste_turbines_reste = self.liste_turbines
        self.turbines_received = True


    def phase_callback(self,msg):
        self.phase = msg.data

    def qrcode_callback(self,msg):
        if(msg.data != 'null'):
            self.qrcode = msg
            self.qrcode_received = True

    def plus_proche_turbine(self):
        ii = 9
        min = 99999999
        for i in range(len(self.liste_turbines_reste)):
            dist = np.sqrt((self.liste_turbines_reste[i].position.x - self.odom.pose.pose.position.x)**2 + (self.liste_turbines_reste[i].position.y - self.odom.pose.pose.position.y)**2)
            if(dist<min):
                min = dist
                ii = i
        return ii
    
    def timer_callback(self):
        if(self.status == 'INITIALIZED'):
            if(self.turbines_received and self.odom_received):
                self.status = 'SEARCH'
                self.turbinesI = self.plus_proche_turbine()
                self.currentgoal = self.liste_turbines_reste[self.turbinesI]

                vect = Point()
                vect.x = (self.currentgoal.position.x - self.odom.pose.pose.position.x)
                vect.y = (self.currentgoal.position.y - self.odom.pose.pose.position.y)
                norm = np.sqrt(vect.x**2 + vect.y**2)
                vect.x = vect.x/norm
                vect.y = vect.y/norm
                self.point.x = self.currentgoal.position.x-vect.x*11
                self.point.y = self.currentgoal.position.y-vect.y*11

                self.currentcameragoal = self.liste_turbines_reste[self.turbinesI]
                self.get_logger().info(self.status)
                self.get_logger().info('going to: "%s"' % self.currentgoal.position)


        if(self.status == 'SEARCH'):
            self.goal_publishers.publish(self.point)

            pointcam = Point()
            pointcam.x = self.currentcameragoal.position.x
            pointcam.y = self.currentcameragoal.position.y
            self.camera_publishers.publish(pointcam)

            if(not self.proche_goal(50)): #pas assez proche pour etre sur que ce soit le bon qrcode
                self.qrcode_received = False

            if(self.qrcode_received): #QR code scanné
                self.qr_publishers.publish(self.qrcode)
                self.get_logger().info('qr code scanned: "%s"' % self.qrcode.data)

                if(len(self.liste_turbines_reste)>0):
                    self.liste_turbines_reste.pop(self.turbinesI)
                if(len(self.liste_turbines_reste)>0):
                    self.turbinesI = self.plus_proche_turbine()
                    self.currentgoal = self.liste_turbines_reste[self.turbinesI]
                    self.currentcameragoal = self.currentgoal

                    vect = Point()
                    vect.x = (self.currentgoal.position.x - self.odom.pose.pose.position.x)
                    vect.y = (self.currentgoal.position.y - self.odom.pose.pose.position.y)
                    norm = np.sqrt(vect.x**2 + vect.y**2)
                    vect.x = vect.x/norm
                    vect.y = vect.y/norm
                    self.point.x = self.currentgoal.position.x-vect.x*11
                    self.point.y = self.currentgoal.position.y-vect.y*11

                    self.get_logger().info('going to: "%s"' % self.currentgoal.position)

            elif(self.proche_goal(13)): #Arrivé mais pas QR code scanné
                vect = Point()
                vect.x = (self.currentgoal.position.x - self.odom.pose.pose.position.x)
                vect.y = (self.currentgoal.position.y - self.odom.pose.pose.position.y)
                self.point.x = self.currentgoal.position.x + vect.x
                self.point.y = self.currentgoal.position.y + vect.y
                self.get_logger().info('turning around turbine')

            if(self.phase == 2):
                self.status = 'RALLY'
                self.get_logger().info(self.status)

        if(self.status == 'RALLY'):
            self.turbinesI = 0
            self.currentgoal = self.liste_turbines[self.turbinesI] 
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
