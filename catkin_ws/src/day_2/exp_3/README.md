# 3. 2-wheeled robot with caster wheel
1. **AIM**: To create a 2-wheeled robot with caster wheel using URDF

2. **METHOD**:
  - Create catkin package: `catkin_create_pkg exp3_postlab`
  - Create urdf files: `cd src/exp3_postlab/src && mkdir urdf && mkdir launch && cd urdf && touch bot.urdf && cd launch && touch gazebo.launch`
  - Add the launch file and urdf definition in this directory
  - Launch using `roslaunch exp3_postlab gazebo.launch`

3. **RESULT**: Successfully simulated/defined parameters for a 2 wheeled robot with a caster wheel via URDF/ROS. Observed output in Gazebo.


