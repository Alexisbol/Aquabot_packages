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

        self.qr_angle_subscription = self.create_subscription(
            Float64,
            '/aquabot/qrcode_angle',
            self.qrcode_angle_callback,
            10
        )


        self.goal_publishers = self.create_publisher(Point,'/aquabot/goal',10)
        self.point_stabilisation_publishers = self.create_publisher(Point,'/aquabot/point_stabilisation',10)
        self.camera_publishers = self.create_publisher(Point,'/aquabot/camera_look_at',10)
        self.qr_publishers = self.create_publisher(String,'/vrx/windturbinesinspection/windturbine_checkup',10)

        self.filtered_distance_range_bearing = self.create_publisher(Float64,'/aquabot/filtered_distance',10)
        self.filtered_angle_range_bearing = self.create_publisher(Float64,'/aquabot/filtered_angle',10)

        self.commande_type_publishers = self.create_publisher(UInt32,'/aquabot/commande_type',10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.previouspahse = 0

        self.odom = Odometry()
        self.odom_received = False
        self.liste_turbines = PoseArray()
        self.turbines_received = False
        self.qrcode = String()
        self.qrcode_received = False

        self.qr_angle = Float64()
        self.qrcode_angle_received = False
        self.phase = UInt32

        self.turbinesI = 0
        self.currentgoal = Point()
        self.currentcameragoal = Point()
        self.point = Point()
        self.liste_turbines_reste = PoseArray()

        # Initialize deques for sliding windows
        self.distance_window = deque(maxlen=7)  # For distances
        self.angle_window = deque(maxlen=5)     # For angles

        # Initialize filter values
        
        # Filtered values
        self.filter_distance = Float64()
        self.filter_angle = Float64()
        self.filter_distance.data = 0.0
        self.filter_angle.data = 0.0

        self.turbine_phase_2 = 0

        self.status = 'INITIALIZED'

        self.commande_type = UInt32()
        self.commande_type.data = 1
        self.commande_type_publishers.publish(self.commande_type)# initialisation : commande de type 1 pour la phase 1

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

    def qrcode_angle_callback(self,msg):
        self.qr_angle = msg
        self.qrcode_angle_received = True

    def ping_callback(self,msg):
        self.ping = msg
        self.ping_received = True
        # Extract values from the message
        distance = msg.params[2].value.double_value
        angle = msg.params[1].value.double_value

        # Add new values to sliding windows
        self.distance_window.append(distance)
        self.angle_window.append(angle)

        self.filter_distance.data = self.median_filter(self.distance_window)
        self.filter_angle.data = self.median_filter(self.angle_window)

        self.filtered_angle_range_bearing.publish(self.filter_angle)
        self.filtered_distance_range_bearing.publish(self.filter_distance)

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
    
    def phase2_la_plus_proche(self):
        #self.get_logger().info('distance = params[2] : "%s"' % self.filter_distance)
        #self.get_logger().info('distances window: "%s"' % self.distance_window)
        yaw = self.odom.pose.pose.orientation.z
        #self.get_logger().info('yaw: "%s"' % yaw)
        #self.get_logger().info('odom x: "%s"' % self.odom.pose.pose.position.x)
        #self.get_logger().info('odom y: "%s"' % self.odom.pose.pose.position.y)
        pos_bateau = np.array([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y])

        distances_eoliennes_from_bateau = np.array([np.linalg.norm(pos_bateau - np.array([turbine.position.x, turbine.position.y])) for turbine in self.liste_turbines])

        #self.get_logger().info('distances_eolienne_from_bateau: "%s"' % distances_eoliennes_from_bateau)

        distances_goal_from_bateau = distances_eoliennes_from_bateau-self.filter_distance.data

        #self.get_logger().info('distances_goal_from_bateau: "%s"' % distances_goal_from_bateau)

        closest_turbine_index = np.argmin(np.abs(distances_eoliennes_from_bateau-self.filter_distance.data))
        #self.get_logger().info('index de l eolienne potentielle: "%s"' % closest_turbine_index)
        return closest_turbine_index

    def median_filter(self, window):
        """
        Apply a median filter to the given sliding window.
        """
        return np.median(window)
    
    def timer_callback(self):
        if self.previouspahse != self.phase:
            self.get_logger().info('phase = "%s"'% self.phase)
        self.previouspahse = self.phase
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
                self.turbine_phase_2 = -1 # reset pour ne pas depasser l'index de la liste
                #norm = np.sqrt(vect.x**2 + vect.y**2)
                #vect.x = vect.x/norm
                #vect.y = vect.y/norm
                
                self.point.x = 0.0
                self.point.y = 0.0
                self.goal_publishers.publish(self.point)

        if(self.status == 'RALLY'):
            findturbine = False
            if findturbine == False:
                self.turbine_phase_2 = self.phase2_la_plus_proche()
                self.currentgoal = self.liste_turbines[self.turbine_phase_2]
                self.currentcameragoal = self.currentgoal
                self.get_logger().info('going to: "%s"' % self.currentgoal.position)

            
            pointcam = Point()
            pointcam.x = self.currentcameragoal.position.x
            pointcam.y = self.currentcameragoal.position.y
            self.camera_publishers.publish(pointcam)

            #self.get_logger().info('phase = "%s"'% self.phase)
            self.goal_publishers.publish(self.point)
            self.point = Point()
            self.point.x = self.currentgoal.position.x + 0.1
            self.point.y = self.currentgoal.position.y
            

            pointcam = Point()
            pointcam.x = self.currentcameragoal.position.x
            pointcam.y = self.currentcameragoal.position.y
            self.camera_publishers.publish(pointcam)

            if (self.proche_goal(50) and self.turbine_phase_2 != self.phase2_la_plus_proche()):
                self.turbine_phase_2 = self.phase2_la_plus_proche()
                self.currentgoal = self.liste_turbines[self.turbine_phase_2]
                self.currentcameragoal = self.currentgoal
                self.get_logger().info('CHANGEMENT DE CIBLE going to: "%s"' % self.currentgoal.position)
            elif (self.proche_goal(30) and self.turbine_phase_2 == self.phase2_la_plus_proche()):
                findturbine = True

            if(not self.proche_goal(50)): #pas assez proche pour etre sur que ce soit le bon qrcode
                self.qrcode_received = False

            if(self.qrcode_received): #QR code scanné
                #utiliser la commande de stabilisation
                self.get_logger().info('----------------------------------')
                self.get_logger().info('angle qr code "%s"' % self.qr_angle.data)
                if (self.qr_angle.data < 0.1 and self.qr_angle.data > -0.1):
                    self.get_logger().info('calcul du point de stabilisation')
                    self.point_stabilisation = Point()
                    vect = Point()
                    vect.x = (self.currentgoal.position.x - self.odom.pose.pose.position.x)
                    vect.y = (self.currentgoal.position.y - self.odom.pose.pose.position.y)
                    norm = np.sqrt(vect.x**2 + vect.y**2)
                    self.point_stabilisation.x = self.currentgoal.position.x - vect.x/norm*11
                    self.point_stabilisation.y = self.currentgoal.position.y - vect.y/norm*11
                    self.status = 'STABILISATION'
                    vect2 = Point()
                    vect2.x = (self.currentgoal.position.x - self.odom.pose.pose.position.x)
                    vect2.y = (self.currentgoal.position.y - self.odom.pose.pose.position.y)
                    norm = np.sqrt(vect2.x**2 + vect2.y**2)
                    vect.x = vect.x/norm
                    vect.y = vect.y/norm
                    self.point.x = self.currentgoal.position.x-vect.x*11
                    self.point.y = self.currentgoal.position.y-vect.y*11
                    self.goal_publishers.publish(self.point)
                    
                    
                
            elif(self.proche_goal(13)): #Arrivé mais pas QR code scanné
                vect = Point()
                vect.x = (self.currentgoal.position.x - self.odom.pose.pose.position.x)
                vect.y = (self.currentgoal.position.y - self.odom.pose.pose.position.y)
                self.point.x = self.currentgoal.position.x + vect.x
                self.point.y = self.currentgoal.position.y + vect.y
                self.get_logger().info('turning around turbine')

        if(self.status == 'STABILISATION'):
            self.commande_type.data = 3 
            self.get_logger().info('------------STABILISATION--------------')
            #self.get_logger().info('angle qr code "%s"' % self.qr_angle.data)
            
            self.commande_type_publishers.publish(self.commande_type) #commande de type 2 pour la phase 2
            
            self.point_ref = Point()                            #point de reference sur l'éolienne pour la stabilisation
            self.point_ref.x = self.currentgoal.position.x
            self.point_ref.y = self.currentgoal.position.y
            self.point_stabilisation_publishers.publish(self.point_ref)
            self.goal_publishers.publish(self.point)
            if (self.phase == 4):
                self.status = 'BONUS'
                self.get_logger().info('self.phase : "%s"' % self.phase)

        if(self.status == 'BONUS'):
            self.get_logger().info('self.phase : "%s"' % self.phase)



                

            




            






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
