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


## Reading, Displaying and Writing an Image


1. Read an image from file (using [cv::imread](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56))
2. Display an image in an OpenCV window (using [cv::imshow](https://docs.opencv.org/4.x/d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563))
3. Write an image to a file (using [cv::imwrite](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#gabbc7ef1aa2edfaa87772f1202d67e0ce))

For details please follow the tutorial [here](https://docs.opencv.org/4.x/db/deb/tutorial_display_image.html)

## Find Chessboard Corners

1. Read an image from file (using [cv::imread](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56))
2. Find chessboard corners using [cv::findChessboardCornersSB](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gadc5bcb05cb21cf1e50963df26986d7c9)
