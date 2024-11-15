# camera_control

Node pour le controle de l'orientation de la caméra

# utilisation

ros2 run camera_control camera_control

# node subscribed to

'/aquabot/odom' Odometry Odometrie donnée par l'EKF

'/aquabot/camera_look_at' Point Le point que la caméra doit viser (coordonnée z non-utilisé)

# node publishing to 

'/aquabot/thrusters/main_camera_sensor/pos' Float64 angle absolue de la caméra

