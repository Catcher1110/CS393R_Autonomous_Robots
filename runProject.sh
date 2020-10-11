#! /bin/bash

gnome-terminal -- "roscore" 
sleep 1
gnome-terminal -- ./runSimulator.sh
gnome-terminal -- ./runWebsocket.sh

gnome-terminal -- ./openWeb.sh
