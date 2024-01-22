#!/bin/sh

ARRAY=1 2 3 4

for num in ${ARRAY[@]}; do
    /usr/bin/python /home/kappa/catkin_ws/src/moveit_tutorials/_scripts/practice/shit.py
    echo $num"回目のループです"
done
