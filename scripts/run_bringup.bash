#!/usr/bin/env bash

while [ -z "$ROBOT_LOG_FOLDER" ]
do
  ROBOT_LOG_FOLDER=$(rosparam get /log_dir)
  echo "ROBOT_LOG_FOLDER = ${ROBOT_LOG_FOLDER}"
done

export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_DISTRO=noetic
export RUNTIME_LOG_FOLDER=${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}
robot_id=phoenix1
echo $robot_id

if [ "$SIMULATION" == "true" ] && [ "$DEVELOPING" == "true" ];
then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source /opt/underlay_ws/setup.bash
  source ${HOME}/robot_ws/install/setup.bash
  source ${HOME}/simulation_ws/install/setup.bash
elif [ "$SIMULATION" == "true" ] && [ "$DEVELOPING" == "false" ];
then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source /opt/underlay_ws/setup.bash
  source /opt/phoenix1/setup.bash
  source /opt/simulation_ws/setup.bash
elif [ "$SIMULATION" == "false" ] && [ "$DEVELOPING" == "true" ];
then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source /opt/underlay_ws/setup.bash
  source ${HOME}/robot_ws/install/setup.bash
else
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source /opt/underlay_ws/setup.bash
  source /opt/phoenix1/setup.bash
fi
source $(rospack find phoenix1_bringup)/scripts/robot_environment_vars.bash

touch ${ROBOT_LOG_FOLDER}/version.txt
echo ${TAG_VERSION} >> ${ROBOT_LOG_FOLDER}/version.txt

session="autonomous_session"

echo "checking for tmux"
tmux has-session -t $session 2>/dev/null
if [ $? = 0 ]; then
  echo "killing tmux"
  tmux detach $session
  # Stopping cleanly previos bag that was recorded
  rostopic pub /record/stop std_msgs/UInt8 "data: 1" --once
  # Stopping ROS before running a new session
  rosnode kill -a;
  killall -9 rosmaster;
  killall -9 roscore;

  tmux kill-session -t $session
fi

tmux new-session -d -s autonomous_session -n base
if [ "$SIMULATION" == "false" ]
then
  tmux send-keys -t autonomous_session:base C-z "stdbuf -o 0 roslaunch phoenix1_bringup base.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_base.txt" Enter
  #TODO: Call this piece of code from a bash script
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

  rosparam set /use_sim_time false

  tmux new-window -t autonomous_session -n rosbags
  tmux select-pane -t 0
  tmux send-keys -t autonomous_session:rosbags C-z "stdbuf -o 0 roslaunch phoenix1_bringup rosbags.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_rosbag.txt" Enter
  tmux select-layout -t autonomous_session:rosbags       tiled

  tmux new-window -t autonomous_session -n diagnostic
  tmux select-pane -t 1
  tmux send-keys -t autonomous_session:diagnostic C-z "stdbuf -o 0 roslaunch phoenix1_bringup wait_for_sensors.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_diagnostics.txt" Enter
  tmux select-layout -t autonomous_session:diagnostic       tiled

elif [ "$SIMULATION" == "true" ]
then
  rosparam set /use_sim_time true
  tmux new-window -t autonomous_session -n simulation
  tmux select-pane -t 0
  tmux send-keys -t autonomous_session:simulation C-z "stdbuf -o 0 roslaunch phoenix1_simulation simulation.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_simulation.txt" Enter
  tmux split-window -t autonomous_session:simulation -h
  tmux select-pane -t 1
  tmux send-keys -t autonomous_session:simulation C-z "stdbuf -o 0 roslaunch amr_embedded_simulator embedded_simulator.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_embedded_simulator.txt" Enter
  tmux select-layout -t autonomous_session:simulation   tiled
fi


tmux new-window -t autonomous_session -n dimmer
tmux select-pane -t 0
tmux send-keys -t autonomous_session:dimmer C-z "stdbuf -o 0 roslaunch ais_dimming_module ais_dimming_module.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_ais_dimming_module.txt" Enter
tmux split-window -t autonomous_session:dimmer -h
tmux select-pane -t 1
tmux send-keys -t autonomous_session:dimmer C-z "rosrun ais_led_strip_controller ais_led_strip_controller_node" Enter
tmux select-layout -t autonomous_session:dimmer   tiled

tmux select-window -t autonomous_session:base
tmux split-window -t autonomous_session:base -h
tmux split-window -t autonomous_session:base -v
tmux select-pane -t 1
tmux send-keys -t autonomous_session:base C-z "stdbuf -o 0 roslaunch phoenix1_bringup wait_for_topics.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_laserscan_multi_merger.txt" Enter


tmux split-window -t autonomous_session:base -h
tmux select-pane -t 2
tmux send-keys -t autonomous_session:base C-z "stdbuf -o 0 roslaunch ais_state_machine safety.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_ais_safety.txt" Enter


tmux select-pane -t 3
tmux send-keys -t autonomous_session:base C-z "stdbuf -o 0 roslaunch phoenix1_bringup description.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_description.txt" Enter

tmux split-window -t autonomous_session:base -h
tmux select-pane -t 4
tmux send-keys -t autonomous_session:base C-z "stdbuf -o 0 roslaunch phoenix_navigation move_base.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_move_base.txt" Enter

tmux split-window -t autonomous_session:base -h
tmux select-pane -t 5
tmux send-keys -t autonomous_session:base C-z "stdbuf -o 0 roslaunch phoenix_navigation rail_navigation.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_rail_navigation.txt" Enter

tmux new-window -t autonomous_session -n version
tmux split-window -t autonomous_session:version -v
tmux select-pane -t 0
tmux send-keys -t autonomous_session:version C-z "stdbuf -o 0 roslaunch ais_job_manager ais_job_manager.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_job_manager.txt" Enter
tmux select-pane -t 1
tmux send-keys -t autonomous_session:version C-z "stdbuf -o 0 roslaunch ais_cru_manager cru_hostname.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_cru_manager.txt" Enter


tmux new-window -t autonomous_session -n statemachine
tmux select-pane -t 0
tmux send-keys -t autonomous_session:statemachine C-z "stdbuf -o 0 roslaunch ais_state_machine phoenix_state_machine.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_state_machine.txt" Enter

tmux new-window -t autonomous_session -n localization
tmux select-pane -t 0
tmux send-keys -t autonomous_session:localization C-z "stdbuf -o 0 roslaunch phoenix1_bringup localization.launch --wait 2>&1 | tee -ai ${HOME}/.ais/logs/${ROBOT_LOG_FOLDER}/${ROBOT_LOG_FOLDER}_localization.txt" Enter


tmux select-window -t autonomous_session:base

tmux select-layout -t autonomous_session:base           tiled
tmux select-layout -t autonomous_session:version        tiled
tmux select-layout -t autonomous_session:statemachine   tiled
tmux select-layout -t autonomous_session:localization   tiled

trap 'sleep infinity' EXIT
