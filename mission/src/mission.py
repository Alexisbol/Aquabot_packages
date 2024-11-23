#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from ros_gz_interfaces.msg import ParamVec
import numpy as np
from collections import deque


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

        # Initialize deques for sliding windows
        self.distance_window = deque(maxlen=10)  # For distances
        self.angle_window = deque(maxlen=3)     # For angles

        # Initialize filter values
        self.filter_angle = 0.0
        self.filter_distance = 0.0

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
        self.get_logger().info('distance = params[2] : "%s"' % self.filter_distance)
        self.get_logger().info('angle = params[1]: "%s"' % self.filter_angle)
        self.get_logger().info('distances window: "%s"' % self.distance_window)
        self.get_logger().info('angles window: "%s"' % self.angle_window)
        yaw = self.odom.pose.pose.orientation.z
        self.get_logger().info('yaw: "%s"' % yaw)
        self.get_logger().info('odom x: "%s"' % self.odom.pose.pose.position.x)
        self.get_logger().info('odom y: "%s"' % self.odom.pose.pose.position.y)
        goal_eolienne_pos = np.array([self.odom.pose.pose.position.x + self.filter_distance * np.cos(np.pi/2-self.filter_angle-yaw),
                                       self.odom.pose.pose.position.y + self.filter_distance * np.sin(np.pi/2-self.filter_angle - yaw)])
        goal_eolienne_pos2 = np.array([self.odom.pose.pose.position.x - self.filter_distance * np.cos(np.pi/2-self.filter_angle-yaw),
                                        self.odom.pose.pose.position.y - self.filter_distance * np.sin(np.pi/2-self.filter_angle - yaw)])
        goal_eolienne_pos3 = np.array([self.odom.pose.pose.position.x + self.filter_distance * np.cos(-self.filter_angle-yaw),
                                        self.odom.pose.pose.position.y + self.filter_distance * np.sin(-self.filter_angle - yaw)])
        goal_eolienne_pos4 = np.array([self.odom.pose.pose.position.x - self.filter_distance * np.cos(self.filter_angle + yaw),
                                        self.odom.pose.pose.position.y - self.filter_distance * np.sin(self.filter_angle + yaw)])
        goal_eolienne_pos5 = np.array([self.odom.pose.pose.position.x + self.filter_distance * np.cos(np.pi/2-self.filter_angle+yaw),
                                       self.odom.pose.pose.position.y + self.filter_distance * np.sin(np.pi/2-self.filter_angle + yaw)])
        goal_eolienne_pos6 = np.array([self.odom.pose.pose.position.x - self.filter_distance * np.cos(np.pi/2-self.filter_angle+yaw),
                                        self.odom.pose.pose.position.y - self.filter_distance * np.sin(np.pi/2-self.filter_angle + yaw)])
        goal_eolienne_pos7 = np.array([self.odom.pose.pose.position.x + self.filter_distance * np.cos(-self.filter_angle+yaw),
                                        self.odom.pose.pose.position.y + self.filter_distance * np.sin(-self.filter_angle + yaw)])
        goal_eolienne_pos8 = np.array([self.odom.pose.pose.position.x - self.filter_distance * np.cos(self.filter_angle - yaw),
                                        self.odom.pose.pose.position.y - self.filter_distance * np.sin(self.filter_angle - yaw)])

        distances_eoliennes_from_goal = np.array([np.linalg.norm(goal_eolienne_pos - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal2 = np.array([np.linalg.norm(goal_eolienne_pos2 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal3 = np.array([np.linalg.norm(goal_eolienne_pos3 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal4 = np.array([np.linalg.norm(goal_eolienne_pos4 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal5 = np.array([np.linalg.norm(goal_eolienne_pos5 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal6 = np.array([np.linalg.norm(goal_eolienne_pos6 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal7 = np.array([np.linalg.norm(goal_eolienne_pos7 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        distances_eoliennes_from_goal8 = np.array([np.linalg.norm(goal_eolienne_pos8 - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])
        closest_turbine_index = np.argmin(distances_eoliennes_from_goal)

        self.get_logger().info('distances_eolienne_from_goal: "%s"' % distances_eoliennes_from_goal)
        self.get_logger().info('distances_eolienne_from_goal2: "%s"' % distances_eoliennes_from_goal2)
        self.get_logger().info('distances_eolienne_from_goal3: "%s"' % distances_eoliennes_from_goal3)
        self.get_logger().info('distances_eolienne_from_goal4: "%s"' % distances_eoliennes_from_goal4)
        self.get_logger().info('distances_eolienne_from_goal5: "%s"' % distances_eoliennes_from_goal5)
        self.get_logger().info('distances_eolienne_from_goal6: "%s"' % distances_eoliennes_from_goal6)
        self.get_logger().info('distances_eolienne_from_goal7: "%s"' % distances_eoliennes_from_goal7)
        self.get_logger().info('distances_eolienne_from_goal8: "%s"' % distances_eoliennes_from_goal8)

        self.get_logger().info('index de l eolienne potentielle: "%s"' % closest_turbine_index)
        return closest_turbine_index

    def median_filter(self, window):
        """
        Apply a median filter to the given sliding window.
        """
        return np.median(window)


    def odom_callback(self,msg):
        self.odom = msg
        self.odom_received = True

    def ping_callback(self,msg):
        self.ping = msg
        self.ping_received = True
        # Extract values from the message
        distance = msg.params[2].value.double_value
        angle = msg.params[1].value.double_value

        # Add new values to sliding windows
        self.distance_window.append(distance)
        self.angle_window.append(angle)

        # Filtered values
        self.filter_distance = self.median_filter(self.distance_window)
        self.filter_angle = self.median_filter(self.angle_window)

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
                self.get_logger().info('--------------V11------------')
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
                vect.x = (turbine.position.x - self.odom.pose.pose.position.x)*1.2

                vect.y = (turbine.position.y - self.odom.pose.pose.position.y)*1.2

                self.currentgoal.position.x = (turbine.position.x + vect.y)
                self.currentgoal.position.y = (turbine.position.y - vect.x)

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
                self.qr_publishers.publish(self.qrcode)
                self.get_logger().info('qr code scanned: "%s"' % self.qrcode.data)
                self.status = 'STABILIZE'
                self.get_logger().info(self.status)

            elif(self.proche_goal(20)): #Arrivé mais pas QR code scanné
                turbine = self.liste_turbines[self.turbinesI]
                vect = Point()
                vect.x = (turbine.position.x - self.odom.pose.pose.position.x)*1.2

                vect.y = (turbine.position.y - self.odom.pose.pose.position.y)*1.2

                self.currentgoal.position.x = (turbine.position.x + vect.y)
                self.currentgoal.position.y = (turbine.position.y - vect.x)

                #self.get_logger().info('turning around turbine')

            
        if(self.status=='STABILIZE'):
            self.get_logger().info('self.phase : "%s"' % self.phase)
            self.get_logger().info('stabilizing')
            
                

            




            





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
