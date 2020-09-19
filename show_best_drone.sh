#!/bin/bash
port=$(($RANDOM + 30000))
# port=65432
DRONE_DIR=$(pwd)
cd $DRONE_DIR
tab=" --tab-with-profile=Default"
tab=" --tab"
options=(--tab --title=Terminal)
timestamp=$1
if [ -z $timestamp ]
then
  timestamp=123
fi

visualize=1
num_servers=1

cmds[1]="pwd; cd $DRONE_DIR; export PYTHONPATH="$DRONE_DIR"; python fly_best_drone.py $timestamp $port"
titles[1]="Server"

cmds[2]="pwd; cd $DRONE_DIR; python visualization/visualization.py $port"
titles[2]="viz"

cmds[3]="pwd; cd $DRONE_DIR; python visualization/vtk_window.py $port"
titles[3]="VTK"



for i in 1 2 3; do
  options+=($tab --title="${titles[i]}"  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"

exit 0