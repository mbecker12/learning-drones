#!/bin/bash
port=$(($RANDOM + 30000))
DRONE_DIR=$(pwd)
cd $DRONE_DIR
tab=" --tab-with-profile=Default"
tab=" --tab"
options=(--tab --title=Terminal)
paramset=$1

visualize=1
num_servers=1

cmds[1]="pwd; cd $DRONE_DIR; export PYTHONPATH="$DRONE_DIR"; python flight/collect_coins.py $port $visualize $num_servers $paramset"
titles[1]="Server"

cmds[2]="pwd; cd $DRONE_DIR; python visualization/visualization.py $port"
titles[2]="viz"




for i in 1 2 3; do
  options+=($tab --title="${titles[i]}"  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"

exit 0