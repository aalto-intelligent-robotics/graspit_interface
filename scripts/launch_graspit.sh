#!/bin/bash


if env | grep -q ^GRASPIT=
then
    echo "Using GRASPIT=" $GRASPIT
else
    export GRASPIT="/home/${USER}/.graspit"
    echo "Using GRASPIT=" $GRASPIT
fi

export GRASPIT_PLUGIN_DIR=$(dirname $(catkin_find --first-only libgraspit_interface.so))
echo "Using GRASPIT_PLUGIN_DIR=" $GRASPIT_PLUGIN_DIR
graspit_simulator -p libgraspit_interface --node_name graspit
