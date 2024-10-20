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


