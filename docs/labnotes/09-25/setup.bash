source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(pwd)/cyclonedds.xml"
alias rosenv='printenv | grep -E "ROS|RMW_IMPLEMENTATION|AMENT|CYCLONEDDS_URI"'
alias syncpubsub="rsync -rcavz $(pwd)/jetbot-ece417/ jetbot@10.0.0.2:~/ece417/"
