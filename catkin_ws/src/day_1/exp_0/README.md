# 0. Pub/Sub to publish and receive name in ROS

1. **AIM**: To print the name via pub/sub

2. **METHOD**: 
  - Create catkin workspace: `mkdir -p catkin_ws/src`
  - Create catkin package: `catkin_create_pkg exp0_postlab rospy roscpp std_msgs`
  - Create scripts: `cd src/exp0_postlab/src && mkdir scripts && cd scripts && touch publisher.py && touch subscriber.py`
  - Add the scripts in this directory
  - Catkin Make: `catkin_make`

3. **RESULT**: Successfully published and received messages via ROS's pubsub transport

