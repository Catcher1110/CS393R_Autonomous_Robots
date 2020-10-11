#! /bin/bash

gnome-terminal -- "roscore" 
sleep .5
gnome-terminal -- ./runSimulator.sh
gnome-terminal -- ./runWebsocket.sh

gnome-terminal -- ./openWeb.sh
