# 5. Image Processing using OpenCV + ROS

1. **AIM**: To find the distance of image from origin in ROS.

2. **METHOD**: 
  - Save your ros distribution in a variable `export ROS_DIST=<noetic|melodic|kinetic>`
  - Install USB Camera support in ROS `sudo apt-get install ros-$ROS_DIST-usb-cam`
  - Install OpenCV `sudo apt-get install opencv-python`
  - Verify Installations are in `$PATH`
  - Start roscore: `roscore`
  - Launch USB Camera: `rosrun usb_cam usb_cam_node`
  - Run the opencv script: `rosrun exp5_postlab image_detect.py`
