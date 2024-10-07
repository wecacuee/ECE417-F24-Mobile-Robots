source /opt/ros/humble/setup.bash || source /opt/ros/humble/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(pwd)/cyclonedds.xml"
alias rosenv='printenv | grep -E "ROS|RMW_IMPLEMENTATION|AMENT|CYCLONEDDS_URI"'
alias syncpubsub='rsync -rcavz $(pwd)/ws/src/jetbot_planning/ jetbot@141.114.195.160:~/ece498/ws/src/jetbot_planning/'
