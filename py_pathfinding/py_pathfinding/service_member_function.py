from custom_interface.srv import Pathfind
from custom_interface.msg import Point
from custom_interface.msg import Path
import rclpy
from rclpy.node import Node
from py_pathfinding.pathFindingcotan import pathfinding
from geometry_msgs.msg import PoseArray  # Pour recevoir les positions des éoliennes
import numpy as np
from scipy.optimize import least_squares

gps_coords = np.array([
    [48.046548317342065, -4.9794483237441804],
    [48.048915152021024, -4.9733800265744375],
    [48.044613214367736, -4.979946020083705]
])

plan_coords = np.array([
    [219.5182, 290.7860],
    [-233.2420, 27.6154],
    [-270.3600, -187.5490]
])

# Fonction pour calculer les erreurs de transformation affine
def affine_transformation(params, gps_coords, plan_coords):
    a, b, c, d, e, f = params
    transformed_points = np.array([
        [a * lat + b * lon + c, d * lat + e * lon + f]
        for lat, lon in gps_coords
    ])
    return (transformed_points - plan_coords).ravel()

# Résoudre pour trouver les paramètres optimaux
initial_guess = [1, 1, 1, 1, 1, 1]
result = least_squares(affine_transformation, initial_guess, args=(gps_coords, plan_coords))
a, b, c, d, e, f = result.x

# Fonction finale pour transformer les coordonnées GPS en coordonnées du repère plan
def gps_to_plan(lat, lon):
    x = a * lat + b * lon + c
    y = d * lat + e * lon + f
    return x, y

# Liste des obstacles existants
liobs = [((120, -50), 35), ((-152, -6), 55), ((110, 135), 50), ((12, -102), 30),
         ((92, 170), 30), ((-92, 176), 40), ((-40, 220), 32), ((-44, -95), 32),
         ((-30, -150), 32)]

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        
        # Créer le service
        self.srv = self.create_service(Pathfind, 'FindPath', self.path_callback)
        
        # Créer la souscription aux positions des éoliennes
        self.subscription = self.create_subscription(
            PoseArray,
            '/aquabot/turbines',
            self.windturbines_callback,
            10
        )
        
        # Stocke les positions des éoliennes dans liobs
        self.liobs = liobs.copy()  # Copie des obstacles de base

    def windturbines_callback(self, msg):
        """Met à jour la liste liobs avec les positions des éoliennes reçues"""
        self.liobs = liobs.copy()  # Réinitialise avec les obstacles de base
        for pose in msg.poses:
            # Extraire les coordonnées x et y
            x, y = pose.position.x, pose.position.y
            # Ajouter chaque éolienne comme obstacle avec un rayon de 10
            if (gps_to_plan(x, y), 8) not in self.liobs :
                self.liobs.append((x, y), 8)
                self.get_logger().info(f'Positions des éoliennes mises à jour :{self.liobs}')

    def path_callback(self, request, response):
        """Service de calcul du chemin"""
        response.path = Path()

        # Calculer le chemin en utilisant la fonction de pathfinding avec la liste d'obstacles mise à jour
        chemin = pathfinding((request.start.x,request.start.y), (request.goal.x,request.goal.y), self.liobs)
        self.get_logger().info('Requête reçue\nDépart: (%d, %d) Arrivée: (%d, %d)' % (request.start.x, request.start.y, request.goal.x, request.goal.y))
        self.get_logger().info(f'chemin : {chemin}')

        # Ajouter chaque point du chemin calculé à la réponse
        for point in chemin:
            point_msg = Point()
            point_msg.x = point[0]  # Assurez-vous que `pathfinding` retourne des tuples ou des objets avec x, y
            point_msg.y = point[1]
            response.path.points.append(point_msg)


        return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
