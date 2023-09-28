# Jetbot camera

Jetbot camera is connected to the Jetson Nano by Camera Serial Interface (CSI) connector. CSI protocol was developed by Mobile Industry Processor Interface (MIPI), hence sometimes specified as MIPI-CSI. CSI is typically faster than USB.

| USB            | MIPI CSI                                |
|----------------|-----------------------------------------|
| 400 MB/s       | 320 MB/s/lane 1280 MB/s (with 4 lanes)  |
| < 5 meters     | <30 cm                                  |
| High           | Low                                     |
| Plug-and-play  | Connect at boot time                    |
| Low            | Medium to High                          |
| Genric drivers | Platform specific drivers               |

### Nvidia CSI camera drivers

Sources:
1. https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera

2. https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/SD/CameraDevelopment/CameraSoftwareDevelopmentSolution.html

Nvidia provides CSI drivers for Jetson Nano through the Libargus camera api. The main command line utility is 
	
	
	jetbot@nano-4gb-jp45:~/ece498$ nvgstcapture-1.0 --mode 1 ----automate --capture-auto
	laptop:~/ece498$ scp jetbot@10.0.0.2:~/ece498/*.jpg .

### ROS camera node

We have ROS camera node available in the ros_deep_learning reposotory that we cloned in the last lecture. Start the docker container

	jetbot@nano-4gb-jp45:~/ece498/data$ docker container start -ia ros-foxy
	root@nano-4gb-jp45:/home/jetbot/ece498# source setup.bash

Find out what nodes are provided by ros_deep_learning package.
	root@nano-4gb-jp45:/home/jetbot/ece498# ros2 pkg executables ros_deep_learning 
	ros_deep_learning detectnet
	ros_deep_learning imagenet
	ros_deep_learning segnet
	ros_deep_learning video_output
	ros_deep_learning video_source

We are interested in the video_source node. ROS nodes are not good at providing help. We have to look at the source code. Open the source either on jupyter lab or on github. https://github.com/dusty-nv/ros_deep_learning/blob/L4T-R35.1.0/src/node_video_source.cpp . Pay attention to the lines with ROS_GET_PARAMETER lines. These are the parameters we can set. It turns out that the bandwidth between laptop and jetbot is pretty limited especially for sending raw images one by one. So we are going to use very small width and height to save band width.

	root@nano-4gb-jp45:/home/jetbot/ece498# ros2 run ros_deep_learning video_source --ros-args -p resource:=csi://0 -p width:=256 -p height:=144 -p framerate:=10 -p flip:=rotate-180

This image can be visualized on the laptop using rviz2
	
	laptop:~/ece498$ source setup.bash
	laptop:~/ece498$ ros2 topic list
	/parameter_events
	/rosout
	/video_source/raw
	laptop:~/ece498$ rviz2


Create a file called ~/.ros/camera.ini with following contents

	# oST version 5.0 parameters


	[image]

	width
	640

	height
	360

	[narrow_stereo]

	camera matrix
	395.086544 0.000000 343.416245
	0.000000 395.599826 188.318197
	0.000000 0.000000 1.000000

	distortion
	-0.311061 0.085598 -0.000239 0.001480 0.000000

	rectification
	1.000000 0.000000 0.000000
	0.000000 1.000000 0.000000
	0.000000 0.000000 1.000000

	projection
	298.174957 0.000000 357.625519 0.000000
	0.000000 366.286163 189.056921 0.000000
	0.000000 0.000000 1.000000 0.000000






	


