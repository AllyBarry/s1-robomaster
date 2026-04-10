#!/bin/bash

docker compose run navigation_demo \
    ros2 launch navigation_demo navigation.launch.py robot_id:=0
