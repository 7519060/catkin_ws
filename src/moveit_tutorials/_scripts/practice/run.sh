#!/bin/bash

# ターミナルを起動してそれぞれのPythonスクリプトを実行します
gnome-terminal --tab --title="Script 1" -- /bin/bash -c "python /home/kappa/catkin_ws/src/moveit_tutorials/_scripts/practice/hello.py" &
gnome-terminal --tab --title="Script 2" -- /bin/bash -c "python /home/kappa/catkin_ws/src/moveit_tutorials/_scripts/practice/shit.py" &
