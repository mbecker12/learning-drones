#!/bin/bash
port=$(($RANDOM + 30000))
cd $HOME/Chalmers/scs/SOCS-Project
tab=" --tab-with-profile=Default"
tab=" --tab"
options=(--tab --title=Terminal)

cmds[1]="pwd; cd $HOME/Chalmers/scs/SOCS-Project; python test_base.py $port 1 2"
titles[1]="Server"

cmds[2]="pwd; cd $HOME/Chalmers/scs/SOCS-Project; python vtk_window.py $port"
titles[2]="VTK"

cmds[3]="pwd; cd $HOME/Chalmers/scs/SOCS-Project; python visualization.py $port"
titles[3]="viz"

for i in 1 2 3; do
  options+=($tab --title="${titles[i]}"  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"

exit 0