# 4. 3 Joint Manipulator with gripper as end effector

1. **AIM**: To Create a manipulator using URDF

2. **METHOD**:
  - Create catkin package: `catkin_create_pkg exp4_postlab`
  - Create urdf/launch files: `cd src/exp4_postlab/src && mkdir urdf && mkdir launch && cd urdf && touch manipulator.urdf && cd ../launch/ && touch gazebo.launch `
  - Add/Copy the files from this directory
  - Catkin Make: `catkin_make`
  - Launch the bot: `roslaunch exp4_postlab gazebo.launch`
  - Note: From ROS Textbook

3. **CODE**:
  - rrbot.xacro:
```xml
<?xml version="1.0"?>
<!-- Revolute-Revolute Manipulator -->
<robot name="rrbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Constants for robot dimensions -->
  <xacro:property name="PI" value="3.14"/>
  <xacro:property name="width" value="0.1" />   <!-- Beams are square in length and width -->
  <xacro:property name="height1" value="2" />   <!-- Link 1 -->
  <xacro:property name="height2" value="1" />   <!-- Link 2 -->
  <xacro:property name="height3" value="1" />   <!-- Link 3 -->
  <xacro:property name="axle_offset" value="0.05" /> <!-- Space between joint and end of beam -->
  <xacro:property name="damp" value="0.7" />    <!-- damping coefficient -->

  <!-- Default Inertial -->
  <xacro:macro name="default_inertial" params="z_value i_value mass">
    <inertial>
      <origin xyz="0 0 ${z_value}" rpy="0 0 0"/>
      <mass value="${mass}" />
      <inertia ixx="${i_value}" ixy="0.0" ixz="0.0"
               iyy="${i_value}" iyz="0.0"
               izz="${i_value}" />
      </inertial>
  </xacro:macro>

  <!-- Import Rviz colors -->
  <xacro:include filename="$(find manipulator)/urdf/materials.xacro" />

  <!-- Import gripper URDF -->
  <xacro:include filename="$(find manipulator)/urdf/gripper.xacro" />  

  <!-- Import Gazebo elements, including Gazebo colors -->
  <xacro:include filename="$(find manipulator)/urdf/rrbot.gazebo" />

  <!-- Used for fixing rrbot frame to Gazebo world frame -->
  <link name="world"/>

  <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height1}"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height1}"/>
      </geometry>
    </collision>

    <xacro:default_inertial z_value="${height1/2}" i_value="1.0" mass="1"/>
  </link>

  <!-- Joint between Base Link and Middle Link -->
  <joint name="joint_base_mid" type="revolute">
    <parent link="base_link"/>
    <child link="mid_link"/>
    <origin xyz="0 ${width} ${height1 - axle_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="${damp}"/>
    <limit effort="100.0" velocity="0.5" lower="-3.14" upper="3.14" />
  </joint>

  <!-- Middle Link -->
  <link name="mid_link">
    <visual>
      <origin xyz="0 0 ${height2/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height2}"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${height2/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height2}"/>
      </geometry>
    </collision>

    <xacro:default_inertial z_value="${height2/2 - axle_offset}" i_value="1.0" mass="1"/>
  </link>

  <!-- Joint between Middle Link and Top Link -->
  <joint name="joint_mid_top" type="revolute"> 
    <parent link="mid_link"/>
    <child link="top_link"/>
    <origin xyz="0 ${width} ${height2 - axle_offset*2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/> 
    <dynamics damping="${damp}"/> 
    <limit effort="100.0" velocity="0.5" lower="-3.14" upper="3.14" />
  </joint>

  <!-- Top Link -->
  <link name="top_link">
    <visual>
      <origin xyz="0 0 ${height3/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height3}"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${height3/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	<box size="${width} ${width} ${height3}"/>
      </geometry>
    </collision>

    <xacro:default_inertial z_value="${height3/2 - axle_offset}" i_value="1.0" mass="1"/>
  </link>

  <transmission name="transmission1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_base_mid">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="transmission2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_mid_top">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor2">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```
  - rrbot_gazebo.launch:
```xml

<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- We resume the logic in gazebo_ros package empty_world.launch, -->
  <!-- changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find manipulator)/worlds/rrbot.world"/>
   
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

  </include>

  <!-- Load the URDF into the ROS Parameter Server -->
  <param name="robot_description"
	 command="$(find xacro)/xacro '$(find manipulator)/urdf/rrbot.xacro'" />

  <!-- Spawn rrbot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
     args="-param robot_description -urdf -model rrbot" />

</launch>
```


4. **RESULT**: Successfully created a 3 joint manipulator, and simulated in Gazebo
