#!/bin/bash

# world 1
gnome-terminal -- roslaunch interiit21 interiit_world.launch world_file:=testing_world1.world&&

# world 2
# gnome-terminal -- roslaunch interiit21 interiit_world.launch world_file:=testing_world2.world&&

# world 3
# gnome-terminal -- roslaunch interiit21 interiit_world.launch world_file:=testing_world3.world&&

cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console



