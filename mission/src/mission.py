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
        self.currentcameragoal = Point()

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
    
    def trouver_turbine_la_plus_proche(self):
        distances_boat_from_goal = []
        angles_boat_from_goal = []
        #for i in range(7): # A verifier ça ne fonctionne surement pas, pb de fréquence
        distances_boat_from_goal.append(self.ping.params[2].value.double_value)
        angles_boat_from_goal.append(self.ping.params[1].value.double_value)
        self.get_logger().info('distance = params[2] : "%s"' % self.ping.params[2].value.double_value)
        self.get_logger().info('angle = params[1]: "%s"' % self.ping.params[1].value.double_value)
        angle_boat_from_goal = np.median(angles_boat_from_goal)
        distance_boat_from_goal = np.median(distances_boat_from_goal)
        self.get_logger().info('distance: "%s"' % distance_boat_from_goal)
        self.get_logger().info('angle: "%s"' % angle_boat_from_goal)
        yaw = self.odom.pose.pose.orientation.z
        self.get_logger().info('yaw: "%s"' % yaw)
        self.get_logger().info('odom x: "%s"' % self.odom.pose.pose.position.x)
        self.get_logger().info('odom y: "%s"' % self.odom.pose.pose.position.y)
        goal_eolienne_pos = np.array([self.odom.pose.pose.position.x + distance_boat_from_goal * np.cos(np.pi/2 - angle_boat_from_goal + yaw),
                                       self.odom.pose.pose.position.y + distance_boat_from_goal * np.sin(np.pi/2 - angle_boat_from_goal + yaw)])

        distances_eoliennes_from_goal = np.array([np.linalg.norm(goal_eolienne_pos - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        closest_turbine_index = np.argmin(distances_eoliennes_from_goal)
        self.get_logger().info('distances_eolienne_from_goal: "%s"' % distances_eoliennes_from_goal)
        self.get_logger().info('index de l eolienne potentielle: "%s"' % closest_turbine_index)
        return closest_turbine_index




    def odom_callback(self,msg):
        self.odom = msg
        self.odom_received = True

    def ping_callback(self,msg):
        self.ping = msg
        self.ping_received = True

    def turbinespose_callback(self,msg):
        self.liste_turbines = msg.poses
        self.turbines_received = True

    def phase_callback(self,msg):
        self.phase = msg

    def qrcode_callback(self,msg):
        if(msg.data != 'null'):
            self.qrcode = msg
            self.qrcode_received = True

    def timer_callback(self):
        if(self.status == 'INITIALIZED'):
            if(self.turbines_received and self.odom_received):
                self.get_logger().info('--------------V8------------')
                self.status = 'SEARCH'
                self.currentgoal = self.liste_turbines[self.turbinesI]
                self.currentcameragoal = self.currentgoal
                self.get_logger().info(self.status)
                self.get_logger().info('going to: "%s"' % self.currentgoal.position)


        if(self.status == 'SEARCH'):

            point = Point()
            point.x = self.currentgoal.position.x + 0.1
            point.y = self.currentgoal.position.y
            self.goal_publishers.publish(point)

            pointcam = Point()
            pointcam.x = self.currentcameragoal.position.x
            pointcam.y = self.currentcameragoal.position.y
            self.camera_publishers.publish(pointcam)

            if(not self.proche_goal(30)): #pas assez proche pour etre sur que ce soit le bon qrcode
                self.qrcode_received = False

            if(self.qrcode_received): #QR code scanné
                self.turbinesI+=1
                self.qr_publishers.publish(self.qrcode)
                self.get_logger().info('qr code scanned: "%s"' % self.qrcode.data)
                
                if(self.turbinesI == len(self.liste_turbines)): #Tous les QR codes scannés
                    self.status = 'RALLY'
                    self.get_logger().info(self.status)
                    self.turbinesI = -1 # reset pour ne pas depasser l'index de la liste
                
                self.currentgoal = self.liste_turbines[self.turbinesI]
                self.currentcameragoal = self.currentgoal

                self.get_logger().info('going to: "%s"' % self.currentgoal.position)

            elif(self.proche_goal(20)): #Arrivé mais pas QR code scanné
                turbine = self.liste_turbines[self.turbinesI]
                vect = Point()
                vect.x = (turbine.position.x - self.odom.pose.pose.position.x)*0.8

                vect.y = (turbine.position.y - self.odom.pose.pose.position.y)*0.8

                self.currentgoal.position.x = (turbine.position.x + vect.x)
                self.currentgoal.position.y = (turbine.position.y + vect.y)

                self.get_logger().info('turning around turbine')

            if(self.phase == 2):
                self.status = 'RALLY'
                self.get_logger().info(self.status)
                self.turbinesI = -1 # reset pour ne pas depasser l'index de la liste

        if(self.status == 'RALLY'):
            if self.turbinesI == -1:
                self.turbinesI = self.trouver_turbine_la_plus_proche()
                self.currentgoal = self.liste_turbines[self.turbinesI]
                self.currentcameragoal = self.currentgoal
                self.get_logger().info('going to: "%s"' % self.currentgoal.position)
                self.status = 'SEARCH' #retour à la recherche à modifier car après le scan on passe à la stabilisation
            
            if(self.proche_goal(15)):
                self.status = 'STABILIZE'
                self.get_logger().info(self.status)
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
