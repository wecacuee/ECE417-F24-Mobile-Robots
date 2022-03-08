---
layout: post
title: ECE417  2022  Lab week  6, Virtual Box to VMware; Car  demo and lane detection
date: 2022-03-07
---

# Summary of steps
{:.no_toc}

* Seed list to be replaced by TOC
{:toc}

# Virtual Box to VMware Workstation player

## Export VirtualBox appliance
  * Power off your virtual machine. Do not save the machine state. Shutdown the virtual machine.

  ```
  sudo shutdown  --poweroff now
  ```
  
  
  * Click on File -> Export Appliance
    ![](vbox-export-appliance.png)

    ![](vbox-export-appliance-2.png)

    ![](vbox-export-appliance-3.png)

    ![](vbox-export-appliance-4.png)
  
  * This will generate an  OVA file. The location of my Ubuntu.ova file  was
   
    `C:\Users\vdhiman\Documents\Ubuntu.ova`
    
    The file size for  me was around 50 GB. 


## Install VMware Workstation player
  * Download non-commercial version of VMware Workstation player from here:
    
    <https://www.vmware.com/go/getplayer-win>
    
  * Install VMWare.
  
## Import VirtualBox in VMware Workstation player

  * "Open" the Ubuntu.ova saved earlier.
  
  ![](vmware-open-ova.png)

  ![](vmware-open-ova-2.png)

  ![](vmware-open-ova-3.png)

  ![](vmware-open-ova-4.png)

  ![](vmware-open-ova-5.png)
  
  * Make sure you are enable the following hardware settings:

     + Edit virtual machine settings
       ![](vmware-settings.png)
  
     + Set  VMware memory to 75% of your available memory.

       ![](vmware-memory.png)
       
     + Set  processors  to 50% of your available processors. Enable VT-x .

       ![](vmware-processor.png)
     
     + Enable 3D acceleration and allow up to 8GB of video memory.
     
       ![](vmware-display.png)
  
  * Play the virtual machine

    * Inside Ubuntu uninstall vbox-guest-additions
    

      ```
      sudo vbox-uninstall-guest-additions
      ```
  
   * Install open-vm-tools
   
    ```
    sudo apt install open-vm-tools open-vm-tools-desktop
    ```

   * Reboot

    ```
    sudo shutdown  --reboot now
    ```

## Check 3D performance

   * Run glmark2 to check  3D performance
   
   ```
   glmark2
   ```
   
   In Virtual Box, I got only 300 FPS but in  VMWare, I get more than  1200 FPS.
   
   ![](glmark2-in-vmware.png)
   
## Check  Lab 5

   * Change directory to location of Lab 5
   ```
   source devel/setup.bash
   catkin_make
   source devel/setup.bash
   roslaunch car_demo demo-light.launch
   ```
  
  *  We will disable software rendering and enable hardware acceleration  in Gazebo.A
     To disable software rendering open `demo-light.launch` and remove the following line:
  
    ```
    <env name="LIBGL_ALWAYS_SOFTWARE"  value="1"  />
    ```
    
    Run gazebo. For  me this did not  work in VMware. 

  * Instead replace the above env line in the `demo-light.launch`. Falling back to OpenGL 2.1 from OpenGL 3.3

    ```
    <env name="SVGA_VGPU10"  value="0"  />
    ```
  * I get 60 FPS on gazebo
  
    ![](gazebo-fps.png)

## Exercise 1

* What FPS did you get with glmark2?
* What FPS did you get with gazebo?

