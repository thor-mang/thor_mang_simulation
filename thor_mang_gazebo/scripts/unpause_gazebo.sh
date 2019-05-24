#!/bin/bash

if [ -z "$1" ]; then
    srv="/gazebo/joints/controller_manager/list_controllers"
else
    srv="/gazebo/$1/joints/controller_manager/list_controllers"
fi

echo ">>> Waiting for '$srv' before unpausing Gazebo."

until rosservice call $srv --wait &>/dev/null ; do
    sleep 1;
done

sleep 5

echo ">>> Unpausing Gazebo..."
rosservice call /gazebo/unpause_physics
echo ">>> Done!"
