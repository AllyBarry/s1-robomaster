#!/bin/bash

docker compose run --remove-orphans -d navigation_demo \
    ros2 launch navigation_demo navigation.launch.py robot_id:=0
