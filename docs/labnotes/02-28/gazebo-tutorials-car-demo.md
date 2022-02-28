---
layout: post
title: ECE417  2022  Lab week  5, Gazebo and  car  demo
date: 2022-02-28
---
# Summary of steps
{:.no_toc}

* Seed list to be replaced by TOC
{:toc}

# Check  VirtualBox settings
Unfortunately  due to global  chip shortage, we will not be able to work  with  real jetbot. So we will  work a car  in a Gazebo  simulation. Gazebo  is 3D simulation   engine and  works best with  graphics card and  3D  acceleration. However, 3D  acceleration   support  in  virtualbox is experimental  and it  may  or  may not  work. In any case, we need a lot of resources for  the virtualbox so make  sure that you have  the following  settings in virtualbox. To changes these  settings  you  will  have to shutdown your Ubuntu virtual machine.

## Recommended  settings
We  should  be  able to   use both the virtual machines  at  the   same   time. So  make sure that you  are not exceeding
   * Laptop  Bios: Make sure VT-x/AMD-V is enabled in your Laptop's bios.
   * Disable  Hyper-V  in  windows: [Windows  instructions](https://docs.microsoft.com/en-us/troubleshoot/windows-client/application-management/virtualization-apps-not-work-with-hyper-v#how-to-disable-hyper-v)
   * Allocate a lot of Memory to VirtualBox:  75% of your available memory in the laptop.

       ![](images/Vbox-memory.png)

   * CPUs:  50% of the  available  CPUs  in  your laptop.

       ![](images/Vbox-processor.png)

   * Acceleration: 

       ![](images/Vbox-acceleration.png)

   * Display  (max  Video memory, enable 3D aceleration): 

       ![](images/Vbox-display.png)
       
In  the  end,  this is how my virtual  machine   settings look like

![](images/Vbox-system-settings.png)


### Checks performance of 3D acceleration from inside Ubuntu  virtual  machine
       
   * Boot the Ubuntu   virtual   machine
   * Make sure Virtualbox Guest additions  is  installed.  Run the   following  command.  
       
       ```
lsmod | grep vbox
```

     I  see  the following  output  on  my machine. You should see vboxsf,vboxguest   and vboxvideo.
     ``` 
$ lsmod | grep vbox
vboxsf                 77824  1
vboxvideo              36864  0
drm_ttm_helper         16384  1 vboxvideo
vboxguest             368640  6 vboxsf
ttm                    69632  3 vmwgfx,vboxvideo,drm_ttm_helper
drm_kms_helper        253952  2 vmwgfx,vboxvideo
drm                   557056  9 vmwgfx,drm_kms_helper,vboxvideo,drm_ttm_helper,ttm
       ```
   * Running  3D tests  to see   performance

   ```
   sudo  apt install   glmark2
   glmark2
   ```

   You should  see  3D rendering of different objects and frame-rates being displayed.
   I get a  "Segmentation fault". It is better if you do  not get it. I get FPS  around 200-400. Higher  is better.
   ```
   $ glmark2 
=======================================================
    glmark2 2021.02
=======================================================
    OpenGL Information
    GL_VENDOR:     VMware, Inc.
    GL_RENDERER:   SVGA3D; build: RELEASE;  LLVM;
    GL_VERSION:    2.1 Mesa 21.0.3
=======================================================
[build] use-vbo=false: FPS: 262 FrameTime: 3.817 ms
[build] use-vbo=true: FPS: 357 FrameTime: 2.801 ms
[texture] texture-filter=nearest: FPS: 372 FrameTime: 2.688 ms
[texture] texture-filter=linear: FPS: 346 FrameTime: 2.890 ms
Segmentation fault (core dumped)
```

![](images/glmark2-horse.png)

# Install Gazebo

Gazebo is popular open-source robotics simulator with  good ROS support. We will install gazebo packages:

``` 
sudo  apt install ros-noetic-gazebo-ros-pkgs
```

####  Check if gazebo is installed. 

You   should be  able  launch  gazebo from a terminal using gazebo  command.

``` 
gazebo
```

![](gazebo-screenshot.png)


## Quick  start
(This is a copy of Gazebo instructions [here](http://gazebosim.org/tutorials?tut=quick_start))
# Run Gazebo

These three steps will run Gazebo with a default world.

1. [Install]( http://gazebosim.org/tutorials?cat=install) Gazebo.

2. Open a terminal. On most Ubuntu systems you can press `CTRL+ALT+t`

3. Start Gazebo by entering the following at the command prompt.

    ~~~
    gazebo
    ~~~

    > Note: The first time you launch gazebo, it will try to download a couple of models so this process may take some time.

# Run Gazebo with a robot


Let's simulate something a bit more interesting by loading a world with a pioneer2dx.

1. Open a terminal and enter the following command.

    ~~~
    gazebo worlds/pioneer2dx.world
    ~~~

    > Note: If you don't have the pioneer2dx model already, Gazebo will download it from the online model database which may take some time.

## Where are the worlds located?

You may have noticed the mysterious `worlds/pioneer2dx.world` argument in the above command. This instructs gazebo to find the `pioneer2dx.world` file, and load it on start.

World files are located in a versioned system directory, for example `/usr/share/gazebo-11` on Ubuntu.  If you have Gazebo 7.0 installed on Ubuntu, in a terminal type the following to see a complete list of worlds.

~~~
ls /usr/share/gazebo-11/worlds
~~~

# Client and server separation

The `gazebo` command actually runs two different executables for you. The
first is called `gzserver`, and the second `gzclient`.

The `gzserver` executable runs the physics update-loop and sensor data
generation. This is the core of Gazebo, and can be used independently of a
graphical interface. You may see the phrase "run headless" thrown about. 
This phrase equates to running only the `gzserver`. An example
use case would involve running `gzserver` on a cloud computer where a user
interface is not needed.

The `gzclient` executable runs a [QT](http://qt-project.org) based user
interface. This application provides a nice visualization of simulation, and
convenient controls over various simulation properties.

Try running each of these executables. Open a terminal and run the server:

~~~
gzserver
~~~

Open another terminal and run the graphical client:

~~~
gzclient
~~~

At this point you should see the Gazebo user interface. You restart the
`gzclient` application as often as you want, and even run multiple
interfaces.

Exit Gazebo  by pressing  Ctrl-C   in the terminal running  gzserver.

#  Vetting appetite:  Car  Demo

What can we do with gazebo?

# Download Car Demo

   * Download car demo from following github repository:
   
   [https://github.com/wecacuee/car_demo/tree/noetic-light](https://github.com/wecacuee/car_demo/tree/noetic-light)
   
   * You can either use git or  click on "Code" -> "Download Zip"
   
   * Move and unzip it to a  catkin workspace
   
   ```
   mkdir -p ~/02-28/catkin_ws/src
   mv ~/Downloads/car_demo-noetic-light.zip ~/02-28/catkin_ws/src
   cd ~/02-28/catkin_ws/src
   unzip car_demo-noetic-light.zip
   mv car_demo-noetic-light car_demo
   ```
   
# Compile Car  demo
   * Install necessary packages

   ```
   sudo apt install  ros-noetic-fake-localization  ros-noetic-robot-state-publisher
   ```
   
   * Compile the catkin  workspace

   ```
   cd ~/02-28/catkin_ws/
   source  /opt/ros/noetic/setup.bash
   catkin_make
   ```
   
   You should see the following  messages in the end:
   ```
[ 88%] Building CXX object car_demo-noetic-light/car_demo/CMakeFiles/PriusHybridPlugin.dir/plugins/PriusHybridPlugin.cc.o
[100%] Linking CXX shared library /tmp/lab-02-28-catkin_ws/devel/lib/libPriusHybridPlugin.so
[100%] Built target PriusHybridPlugin
```

#  Attempt  1: Run car demo with 3D acceleration

We will try  to run car  demo with 3D acceleration. This may  or may not work on your machine.  On  my machine  it  did not. If this works then you should stick with it.

   * Roslaunch  demo-light.launch

   ```shellsession
   source devel/setup.bash
   roslaunch car_demo demo-light.launch
   ```

