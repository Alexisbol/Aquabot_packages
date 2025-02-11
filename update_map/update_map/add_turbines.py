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

        # Obtenir le répertoire de base où le script est exécuté
        self.base_path = os.path.dirname(os.path.abspath(__file__))
        self.get_logger().info(f"base path  : {self.base_path}")


        # Construire les chemins relatifs
        self.map_path = self.base_path.replace('update_map/lib/python3.10/site-packages/update_map', 'nautilus_launch/share/nautilus_launch/params/map_400.png')
        self.updated_map_path = self.base_path.replace('update_map/lib/python3.10/site-packages/update_map', 'nautilus_launch/share/nautilus_launch/params/map_updated_400.png')
        self.get_logger().info(f"nouveau path  : {self.updated_map_path}")

        #self.updated_map_path = "map_updated_400.png"

    def sub_cb(self,msg):
        self.get_logger().info("turbines pos get")
        if self.map_updated == False :
            turbines = []
            for pos in msg.poses:
                p=pos.position
                turbines.append((int(p.x+400),int(-p.y+400)))


            img = cv2.imread(self.map_path) 
            if img is None:
                self.get_logger().error("Erreur : Impossible d'ouvrir l'image.")

            for p in turbines:
                self.get_logger().info(f"Type de p : {type(p)}, Valeur : {p}")

                cv2.circle(img,p,20,(150,150,150),-1)
                cv2.circle(img,p,15,(75,75,75),-1)
                cv2.circle(img,p,10,(0,0,0),-1)

            if cv2.imwrite(self.updated_map_path, img):
                self.get_logger().info(f"Image mise à jour enregistrée dans : {self.updated_map_path}")

            if not cv2.imwrite(self.updated_map_path, img):
                self.get_logger().error("Erreur lors de la sauvegarde de l'image.")

            self.map_updated = True

def main():
    rclpy.init()

    add_turbines = Add_turbines()
    rclpy.spin(add_turbines)
    rclpy.shutdown()

if __name__=='__main__':
    main()
