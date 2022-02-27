---
layout: post
title: ECE417  2022  Lab week  5, Virtual Jetbot
date: 2022-02-28
---
# Summary of steps
{:.no_toc}

* A markdown unordered list which will be replaced with the ToC
{:toc}

# Create  a new  virtual  machine for  jetbot
Unfortunately  due to global  chip shortage, we will not be able to work  with  real jetbot. So we will  create  a   virtual jetbot in another virtual machine. If you have  enough space you  on your machine, the easiest   thing is to clone  the Ubuntu virtual machine.  If  not, repeat  the steps of first lab to create  a virtual machine, call  it jetbot.
### Recommended  settings
We  should  be  able to   use both the virtual machines  at  the   same   time. So  make sure that you  are not exceeding
   * Memory:  25% of your available memory in the laptop.
   * CPUs:  25% of the  available  CPUs  in  your laptop.
   * Disk space: Chose the   dynamic version of  virtual disk  in  VDI  format,  with  32  GB of max disk space.
   * Choose  minimal  install  of  Ubuntu  with   drivers
   *  Choose machine name: jetbot  . If  you  cloned  the machine. You can  change  the  name of the  machine  by using the the following command
   ```shellsession
   $ sudo hostname jetbot
   ```
   *  Choose username : jetbot, Password: jetbot
   If you cloned  the  machine. You  can add  a  user using  the  command
   ```shellsession
   $ sudo adduser jetbot
   ```

# Install Gazebo

Since we  do  not  have a  real robot,  we  will  need to install a  robot simulator. Gazebo is popular open-source robotics simulator with  good ROS support. We will install gazebo packages:

``` shellsession
$ sudo  apt install ros-noetic-gazebo-ros
```

####  Check if gazebo is installed. 

You   should be  able  launch  gazebo from a terminal using gazebo  command.

``` shellsession
$ gazebo
```

![](gazebo-screenshot.png)

# Import gazebo models

On the  left hand pane,  click on Insert >  "https://modelsim.gazebo.org"  >  "Ambulance".  Click  somewhere in the  world to drop the "Ambulance" model.

![](gazebo-insert-button.png)
![](gazebo-models-button.png)
![](gazebo-ambulance-button.png)
![](gazebo-ambulance-dropped.png)

#### Check gazebo models directory

Make sure the  `/home/<username>/.gazebo/models` directory has ambulance in  it.
![](gazebo-models-directory-created.png)

#### Check the gazebo models directory in terminal

``` shellsession
$ ls  ~/.gazebo/models
```

You must  see ambulance directory.
``` shellsession
$ ls   ~/.gazebo/models/
ambulance
```

Close gazebo.

#  Download jetbot  gazebo models

Go  to  melodic  branch  of jetbot_ros  repository:

[https://github.com/dusty-nv/jetbot_ros/tree/melodic](https://github.com/dusty-nv/jetbot_ros/tree/melodic)

Click on "Code" and press "Download ZIP".



