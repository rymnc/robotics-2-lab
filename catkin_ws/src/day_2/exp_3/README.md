# 3. 2-wheeled robot with caster wheel
1. **AIM**: To create a 2-wheeled robot with caster wheel using URDF

2. **METHOD**:
  - Create catkin package: `catkin_create_pkg exp3_postlab`
  - Create urdf files: `cd src/exp3_postlab/src && mkdir urdf && mkdir launch && cd urdf && touch bot.urdf && cd ../launch && touch gazebo.launch`
  - Add the launch file and urdf definition in this directory
  - Launch using `roslaunch exp3_postlab gazebo.launch`

3. **CODE**
  - bot.urdf:
```xml
<?xml version='1.0'?>
<robot name="exp3">

  <gazebo>
    <static>False</static>
  </gazebo>


  <link name="dummy"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.7 0.7 0.30"/>
      </geometry>
    </visual>
    <!-- Base collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.7 0.7 0.30"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="7"/>
      <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.25" iyz="0.0" izz="0.15"/>
    </inertial>

    <!-- Caster Wheel -->
    <visual name="caster">
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
      <origin xyz="0.4 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.10" />
      </geometry>
    </visual>
    <!-- Caster collision, mass and inertia -->
    <collision>
      <origin xyz="0.4 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.10" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>

  </link>

  <gazebo reference="base_link">
    <material>Gazebo/RedGlow</material>
    <pose>0 0 3 0 0 0</pose>
  </gazebo>

  <joint name="dummy_joint" type="fixed">
    <parent link="dummy"/>
    <child link="base_link"/>
  </joint>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
    <!-- Right Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <gazebo reference="right_wheel">
    <material>Gazebo/RedGlow</material>
  </gazebo>

  <!-- Right Wheel joint -->
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <material name="blue">
        <color rgba="0.0 0.0 1 1"/>
      </material>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
    <!-- Left Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <gazebo reference="left_wheel">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- Left Wheel joint -->
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```
  - gazebo.launch:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  
  <include file="$(find gazebo_ros)/launch/empty_world.launch">   
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
  </include>
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="screen"
     args="-file $(find exp3_postlab)/urdf/bot.urdf -urdf -z -2.82 -model exp3" />

</launch>
```
4. **RESULT**: Successfully simulated/defined parameters for a 2 wheeled robot with a caster wheel via URDF/ROS. Observed output in Gazebo.


