# 4. 3 Joint Manipulator with gripper as end effector

1. **AIM**: To Create a manipulator using URDF

2. **METHOD**:
  - Create catkin package: `catkin_create_pkg exp4_postlab`
  - Create urdf/launch files: `cd src/exp4_postlab/src && mkdir urdf && mkdir launch && cd urdf && touch manipulator.urdf && cd ../launch/ && touch gazebo.launch `
  - Add the urdf and launch files from this directory
  - Catkin Make: `catkin_make`
  - Launch the bot: `roslaunch exp4_postlab gazebo.launch`

3. **RESULT**: Successfully created a 3 joint manipulator, and simulated in Gazebo
