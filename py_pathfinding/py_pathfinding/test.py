import numpy as np
from scipy.optimize import least_squares

# Données des coordonnées GPS (latitude, longitude) et coordonnées dans le repère plan (x, y)
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

# Test avec les coordonnées GPS
for i, (lat, lon) in enumerate(gps_coords):
    x, y = gps_to_plan(lat, lon)
    print(f"Éolienne {i+1} - GPS: ({lat}, {lon}) -> Repère Plan: (x={x:.4f}, y={y:.4f})")


print(gps_to_plan(48.04700535961105, -4.975036409149975))