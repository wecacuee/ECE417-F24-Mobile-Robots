---
layout: post
title: ECE417  2022  Lab 9, Detect chessboard hints
date: 2022-04-11
usemathjax: true
---

# Summary of steps
{:.no_toc}

* Seed list to be replaced by TOC
{:toc}


# Creating a new ROS package for final project

We learned about creating a ROS package in [turtlesim tutorials](http://wiki.ros.org/ROS/Tutorials/CreatingPackage). Create a new ROS package for the final project. Name it appropriately, so that it reflects the topic of your project.

# OpenCV

OpenCV stands for open source computer vision library. We have used OpenCV to implement loading images and converting them from ROS messages to Eigen matrices. If you look back at the template code of Birds Eye View Lab and Homography lab, you can find code using OpenCV.

## OpenCV and CMake

You will need to find the package OpenCV using

``` cmake
find_package(OpenCV 4.2 REQUIRED)
```

And then include_directories and target_link_libraries with the target

``` cmake
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(some_target_name_here 
other_libraries_names_here like ${catkin_LIBRARIES}, Eigen3::Eigen
${OpenCV_LIBRARIES}
)
```

## Reading, Displaying and Writing an Image


1. Read an image from file (using [cv::imread](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56))
2. Display an image in an OpenCV window (using [cv::imshow](https://docs.opencv.org/4.x/d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563))
3. Write an image to a file (using [cv::imwrite](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce))

For details please follow the tutorial [here](https://docs.opencv.org/4.x/db/deb/tutorial_display_image.html)

## Find Chessboard Corners

1. Read an image from file (using [cv::imread](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56))
2. Find chessboard corners using [cv::findChessboardCornersSB](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gadc5bcb05cb21cf1e50963df26986d7c9)

You might find this tutorial helpful:
<https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html>

## Template code for the project

1. The template code (a catkin workspace) for the project is [here](https://github.com/wecacuee/ECE417-Mobile-Robots/tree/master/docs/labnotes/04-11-detect-chessboard/code). All the files linked below are in that folder.

2. You have to run the cv_camera_node (or usb_cam_node) to publish the image topics at `/cv_camera/image_raw`

   ``` shellsession
   rosrun cv_camera  cv_camera_node   _image_width:=800 _image_height:=600 _frame_id:=camera __name:=cv_camera
   ```

3. While the above node is running, you can catkin_make the provided ROS node try fixing the math in [this file](https://github.com/wecacuee/ECE417-Mobile-Robots/blob/master/docs/labnotes/04-11-detect-chessboard/code/catkin_ws/src/camera_calibration_3d/src/camera_calibration.cpp)

   ``` shellsession
   cd "path where you downloaded the above files"/catkin_ws
   catkin_make
   source devel/setup.bash
   rosrun camera_calibration_3d camera_calibration
   ```
   
   The camera_calibration node will cycle through "sliding windows" in the image to find the chessboard. You might have to tweak some parameters to make it work on your image. I tested it on [this image](https://github.com/wecacuee/ECE417-Mobile-Robots/blob/master/docs/labnotes/04-11-detect-chessboard/code/catkin_ws/src/camera_calibration_3d/data/two-chessboard.jpg)
   
4. If you like launch files, [here is a launch file](https://github.com/wecacuee/ECE417-Mobile-Robots/blob/master/docs/labnotes/04-11-detect-chessboard/code/catkin_ws/src/camera_calibration_3d/launch/camera_calibration_3d.launch) to run multiple nodes.
