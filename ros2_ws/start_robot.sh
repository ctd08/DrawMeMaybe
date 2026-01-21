#!/bin/bash
#Mit diesem Script wird das Laden der Datei "esternal.urp" und das Drücken auf den Playbutton erledig
echo "Die Verbindung zum Roboter wird gestartet ..."
echo "Stellen Sie sicher, dass der Roboter auf Extern-Modus steht!"

#Ros-Variablen in die aktuelle Shell laden
source ./install/setup.bash

#Die Verbingung mit dem Roboter starten
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.3

echo "Die Verbindung ist aktiviert"

#Das Programm für die externe Verbingung laden (external.urp)
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: external.urp"``
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {} #Startbutton drücken

echo "Die ExternalUpr steht, der UR5e-Roboter ist startbereit"

