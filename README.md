# Aquabot_packages

Ce repo github a pour but d'héberger les packages contenenant les noeuds ROS2.
Il devrait être clonné dans le dossier /src de /ros2_ws . Et /ros2_ws est à côté du dossier /vrx_ws
On aura le workspace sirhena et le notre séparer ce qui sera plus simple pour build le nôtre plus facilement.

Chacun peut créer un package et travailler dessus. 

# Pour travailler sur un noeud

faire une nouvelle branche avec le nom du noeud 

Ajouter ses fichiers sur la nouvelle branche(voir vision_node)

Commit pour sauvegarder en local et push pour envoyer la sauvegarde sur le github

Ensuite, envoyer une pull request de la branch avec le noeud vers la branche main

# Pour installer l'environnement

```
git clone git@github.com:Alexisbol/Aquabot_packages.git
cd ~/home/nom_utilisateur/nom_workspace
colcon build
source install/setup.bash
```

Pour simplifier la compilation et l'execution du fichier de setup on peut ajouter les lignes suivante dans le bashrc.
Pour ouvrir le fichier :
```
gedit ~/.bashrc
```

Ajouter les lignes suivantes :
```
# Add Alias
alias ecn_build='cd ~/ros2_ws && colcon build --merge-install'
alias ecn_source='source ~/ros2_ws/install/setup.bash'
```
Il suffit de taper `ecn_build` quand il y a des changements dans le code pour le recompiler.

Si on veux utiliser un noeud de Aquabot_packages, il suffit de faire `ecn_source` pour que le terminal comprenne les commandes des noeuds du package.



# Pour basculer entre les branches

```
git checkout nom_branche
```

# Pour installer les dépendances de l'ekf

```
sudo apt install ros-humble-robot-localization
```

# run the camera_control node :

1) Start the simulation
```
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```

2) Start rqt ou rviz2
rqt est plus simple 
```
rviz2
```
```
rqt
```

add the image of the camera

3) Start the keyboard teleoperation

```
ros2 run aquabot_python teleop_keyboard.py
```

4) Start EKF
Il s'agit d'un noeud de Aquabot_packages donc il faut pas oublier d'executer le fichier de setup dans le terminal
```
cd ros2_ws
source install/setup.bash
```
Puis on peut lancer l'ekf :
```
ros2 launch aquabot_ekf ekf_launch.py
```

5) Start camera_control node
Il s'agit d'un noeud de Aquabot_packages donc il faut pas oublier d'executer le fichier de setup dans le terminal

```
cd ros2_ws
source install/setup.bash
```
```
ros2 run camera_control camera_control
```

6) Send position to look at :
Le noeud de control de la camera à besoin d'un point de référence à regarder, on peut lui en envoyer un sur le topic /aquabot/camera_look_at avec la commende suivante :
```
ros2 topic pub /aquabot/camera_look_at geometry_msgs/Point "{x: 1000.0, y: 1000.0, z: 0.0}"
```
