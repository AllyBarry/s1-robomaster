#!/bin/bash
# declare -a arr=(2 3)
# for i in "${arr[@]}"
# 10.3.1.$i
# do
rsync -a --progress --exclude paper --exclude docs --exclude setup . rasppiuser@192.168.1.16:/home/rasppiuser/Robot
# done

