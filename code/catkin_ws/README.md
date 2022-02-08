# Usage

``` shellsession

. /opt/ros/noetic/setup.bash
rosdep install --from-paths src
catkin_make
. devel/setup.bash
roslaunch camera_model_example camera_model_example.launch
```
