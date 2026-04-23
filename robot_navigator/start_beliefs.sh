#!/bin/bash

./run_belief.sh 0 --with-waypoints --peers 1,2
./run_belief.sh 1 --with-waypoints --peers 0,2
./run_belief.sh 2 --with-waypoints --peers 1,0
