---
layout: post
title: ECE417  2022  Lab 8, Computing homography
date: 2022-04-04
usemathjax: true
---

# Summary of steps
{:.no_toc}

* Seed list to be replaced by TOC
{:toc}

# Removing perspective

## Install required packages

``` shellsession
sudo apt install g++ cmake libeigen3-dev make libopencv-dev
```

## Create a directory for todays lab

``` shellsession
mkdir -p ~/04-04/code
```

## Download template code

1. Download [`CMakeLists.txt`](https://github.com/wecacuee/ECE417-Mobile-Robots/blob/master/docs/labnotes/04-04-homography/code/CMakeLists.txt) to `~/04-04/code/CMakeLists.txt`.

2. Download [`homography.cpp`](https://github.com/wecacuee/ECE417-Mobile-Robots/blob/master/docs/labnotes/04-04-homography/code/homography.cpp) and save to `~/04-04/code/homography.cpp`.

3. Download [`data/removing-perspective-distortion.png`](https://github.com/wecacuee/ECE417-Mobile-Robots/blob/master/docs/labnotes/04-04-homography/data/removing-perspective-distortion.png) and save to `~/04-04/code/data/removing-perspective-distortion.png`.

## Compile

``` shellsession
mkdir -p ~/04-04/code/build
cd ~/04-04/code/build
cmake ..
make
```

## Run

Once you run the following command, a window with image will show up.

``` shellsession
cd ~/04-04/code/build
./homography ../data/removing-perspective-distortion.png
```

![](media/opencv-window-input-points.png)

Left click on marked points in the following order left top, left bottom, right top, right bottom. Then press any key to quit. This will record the coordinates of the marked points. In general you can chose to click on any rectangular thing in the image. Bigger the rectangular thing, better is the H estimation.

![](media/terminal_output.png)
...
![](media/terminal_output_end.png)

## Fill in correct formulas

Fill in `homography.cpp` with correct formula from the exam or 03/25 lecture.

Once you fill in everything right, you should see this as the final image:

![](media/opencv-window-output-image.png)

### Submission 1

Submit the homography.cpp file

## Get a picture with perspective effect

Note that we do not  need camera calibration matrix to complete this process.

Click a picture of the lab so that windows or cupboards or any other rectangular
object appears with a perspective effect (using any camera mobile or laptop).
Perspective effect is when parallel lines in the 3D world are not parallel,
instead converging in the image.


Somehow (by emailing it to yourself?) get the image to the linux and save the image as `~/04-04/code/data/removing-perspective-distortion.png`.


### Submission 2

Repeat the process to remove distortion. Submit the screenshot of image with and without perspective effect.

