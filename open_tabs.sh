#!/bin/bash

# tab=" --tab-with-profile=Default"
# options=(--tab --title=Terminal)

# cmds[1]="python main_loop.py"
# titles[1]="Server"

# cmds[2]="python vtk_window.py"
# titles[2]="VTK"

# cmds[3]="python visualization.py"
# titles[3]="viz"

# for i in 1 2 3; do
#   options+=($tab --title="${titles[i]}"  -e "bash -c \"${cmds[i]} ; bash\"" )          
# done

gnome-terminal --tab --command "echo 1234" --tab --command "echo 5678" --command "sleep 5"


