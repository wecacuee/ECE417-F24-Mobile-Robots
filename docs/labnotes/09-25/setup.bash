source /opt/ros/foxy/setup.bash || source /opt/ros/foxy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(pwd)/cyclonedds.xml"
alias rosenv='printenv | grep -E "ROS|RMW_IMPLEMENTATION|AMENT|CYCLONEDDS_URI"'
