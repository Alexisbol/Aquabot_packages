# Aquabot_packages

Ce repo github a pour but d'héberger les packages contenenant les noeuds ROS2.
Dans l'idée il devrait être clonner à côté du dossier /vrx_ws
On aura le workspace sirhena et le notre séparer ce qui sera plus simple pour build le nôtre plus facilement.

Chacun peut créer un package et travailler dessus. 

# Pour travailler sur un noeud

faire une nouvelle branche avec le nom du noeud 

Ajouter ses fichiers sur la nouvelle branche(voir vision_node)

Commit pour sauvegarder en local et push pour envoyer la sauvegarde sur le github

Ensuite, envoyer une pull request de la branch avec le noeud sur la branche main

# Pour installer l'environnement

```
git clone git@github.com:Alexisbol/Aquabot_packages.git
cd ~/home/nom_utilisateur/nom_workspace
colcon build
source install/setup.bash
```

# Pour basculer entre les branches

```
git checkout nom_branche
```

# Pour installer les dépendances de l'ekf

```
sudo apt install libgeographic-dev
sudo apt install ros-humble-robot-localization
```

# run the camera_control node :

1) Start the simulation
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```

2) Start rviz2
```
rviz2
```
add the image of the camera

3) Start the keyboard teleoperation
```
ros2 run aquabot_python teleop_keyboard.py
```
4) Start EKF
```
cd ros2_ws
source install/setup.bash
ros2 launch aquabot_ekf ekf_launch.py
```
5) Start camera_control node
```
cd ros2_ws
source install/setup.bash
ros2 run camera_control camera_control
```
6) Send position to look at :
```
ros2 topic pub /aquabot/camera_look_at geometry_msgs/Point "{x: 1000.0, y: 1000.0, z: 0.0}"
```
