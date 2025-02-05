import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray 
import numpy as np
import cv2
import os

class Add_turbines(Node):



    def __init__(self):
        super().__init__('add_turbines')

        self.sub = self.create_subscription(PoseArray,'/aquabot/turbines',self.sub_cb,10)

        self.map_updated = False


    def sub_cb(self,msg):
        self.get_logger().info("turbines pos get")
        if self.map_updated == False :
            turbines = []
            for pos in msg.poses:
                p=pos.position
                turbines.append((int(p.x-200),int(p.y-200)))

            img = cv2.imread("/home/mathijs/ros2_ws/install/nautilus_launch/share/nautilus_launch/params/map_400.png")
            if img is None:
                self.get_logger().info("image non ouverte")

            for p in turbines:
                self.get_logger().info(f"Type de p : {type(p)}")

                cv2.circle(img,p,20,(255,255,255),-1)

            cv2.imwrite("/home/mathijs/ros2_ws/install/nautilus_launch/share/nautilus_launch/params/map_400_updated.png",img)
            self.map_updated = True

def main():
    rclpy.init()

    add_turbines = Add_turbines()
    rclpy.spin(add_turbines)
    rclpy.shutdown()

if __name__=='__main__':
    main()
