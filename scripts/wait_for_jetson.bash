#!/usr/bin/env bash
echo "Wait for jetson script"
while :
do
    if ping -c 1 -w 1 uv-vision-module  &> /dev/null
    then
        echo "Jetson is ready"
        ssh phoenix-xavier@uv-vision-module "docker exec -i vm2-binder-dev bash -c '/opt/vm2/robot_ws/src/amr_human_detection/scripts/run_phoenix.bash'"
        break
    fi
done
