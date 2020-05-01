# Petemoya Guilain

Guide

Requis :
Simulation:
Le logiciel CoppeliaSim  (version EDU) pour linux sur votre machine: https://www.coppeliarobotics.com/downloads

La librairie Open CV : https://www.learnopencv.com/install-opencv-4-on-ubuntu-18-04/

CMake : Taper dans votre terminal : 

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install cmake


Ensuite, télécharger le dossier directement depuis le site , ou le cloner en appliquant cette commande dans le terminal:

    git clone https://github.com/GuilainP/StageM1.git

Lancer CoppeliaSim et ouvrir la scène e-puck.ttt contenue dans le dossier StageM1/ 

Se rendre dans le dossier StageM1/build/ et faire les commandes suivantes:

    cmake ..
    make

Pour lancer l'application taper :

    ./apps/robot


Application : 

Un dossier est crée pour les données récoltées:
![](logs_tree.png)

Cette application permet de faire bouger l' e-puck et de récolter différentes observations.

Pour arrêter le programme , taper ctrl + c dans le terminal;



