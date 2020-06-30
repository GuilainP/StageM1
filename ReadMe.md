# Guide d'installation

### Prérequis :

* Pour la simulation, le logiciel [CoppeliaSim](https://www.coppeliarobotics.com/downloads)  (version EDU) pour linux.
```sh
$ # extraire le fichier
$ tar -xf CoppeliaSim_Edu_V4_0_0_Ubuntu18_04.tar.xz
$ # déplacement dans le dossier home/
$ mv CoppeliaSim_Edu_V4_0_0_Ubuntu18_04 ~
$ # lancement de l'application
$ ~/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/coppeliaSim.sh
```

* La librairie Open CV
```sh
$ sudo apt install libopencv-dev
```

* Les logiciel CMake et Git
```sh
$ sudo apt install cmake git
```


### Installation

Télécharger le dossier directement depuis le site , ou le cloner en appliquant cette commande dans le terminal:

    $ git clone https://github.com/GuilainP/StageM1.git

Lancer CoppeliaSim et ouvrir la scène `e-puck.ttt` contenue dans le dossier StageM1/ 

Se rendre dans le dossier StageM1/build/ et faire les commandes suivantes:
```sh
$ cmake ..
$ make
```
Pour lancer l'application taper :
```sh
$ # lancer la simulation
$ ./apps/robot setWheelCmd 127.0.0.1 5 -5 
$ # pour un robot
$ ./apps/robot setWheelCmd 192.168.1.8 50 -50  
```

Application : 

Un dossier est crée pour les données récoltées:

![](logs_tree.png)

Cette application permet de faire bouger l' e-puck et de récolter les mesures des différents capteurs.

Pour arrêter le programme , taper ctrl + c dans le terminal.

L' e-puck de la scène a été modifié pour mieux correspondre au robot réel.
Vous avez la liberté de modifier la scène pour faire vos propres observations.

Ressources utilisées :

E-puck -puck version 1 :    
* https://www.gctronic.com/doc/index.php/E-Puck
* https://gitlab.com/umrob/epuck-client

E-puck version 2 :
* https://www.gctronic.com/doc/index.php/e-puck2
* https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development
* https://github.com/gctronic/epuck_driver_cpp/blob/e-puck2_wifi/src/epuck2_driver_cpp.cpp

E-puck CoppeliaSim (Vrep) :
* https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm
* https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm
