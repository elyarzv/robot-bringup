#!/usr/bin/env bash
echo "Wait for topics script"
for test_topic_name in {"FrontLidar/scan_topic","BackLidar/scan_topic"}
do
  echo "Check if ${test_topic_name} is published:"
  ## sleep in bash for loop ##
  export status=$(/bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && rostopic info ${test_topic_name}" | grep "^Type: " | wc -l)
  while [[ $status -eq 0 ]]
  do
    echo "Waiting for ${test_topic_name}"
    sleep 1
    export status=$(/bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && rostopic info ${test_topic_name}" | grep "^Type: " | wc -l)
    echo $status
  done
done

roslaunch phoenix1_bringup laserscan_multi_merger.launch
