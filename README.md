# Dobot Magician ROS control for speech recognition,reading, writing and drawing
A repository that contains ROS control of the dobot magician for read, write and draw operations. It integrates speech recognition and text detection, giving a starting point for users to work with the dobot magician via ROS.

This repository has been successfully implemented using ROS-melodic on ubuntu 18.04. Feel free to try it on kinetic as well and it should work.

The dobot magician ros demo can be found [here](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)

Steps to run the code:
- Install Ununtu 18.04 on your system via dual boot
- Install ROS-melodic following the tutorial given [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Then run the following commands
  -  cd 
  -  git clone https://github.com/Niegil-Francis/dobot_ws.git
  -  cd dobot_ws
  -  catkin_make
- Then connect the dobot magician via USB and run the following two commands on separate terminals
  - roscore
  - rosrun dobot DobotServer ttyUSB0
- Replace ttyUSB0 with whichever port the dobot magician is connected to - try the command: *dmesg | grep tty*  before and after connecting the usb to see which port you need to use
- Check if the connection is successful, if not run: *sudo chmod a+rw /dev/ttyUSB0* again replacing the port according to whichever port you are connected to

- Then run the following two commands on separate terminals 
  - rosrun dobot Topic
  - python3 dobot_ws/src/dobot/scripts/Writing_Drawing.py --camera 0

Note: Each time the dobot magician turns on, the coordinates are reset randomly. If you have windows, download the dobot api for windows given [here](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download), power on the dobot magician and click the home button on the api. Keep the dobot magician on and switch over to ubuntu and the axis will be correctly aligned. I also strongly recommend going through the communication protocol found in the same link.

Sources: [https://hotblackrobotics.github.io/en/blog/2018/06/29/ros-dobot/](https://hotblackrobotics.github.io/en/blog/2018/06/29/ros-dobot/)
