# Chapter 8: URDF for Humanoids

The Unified Robot Description Format (URDF) is an XML-based format for describing robot models in ROS. This chapter covers how to create, modify, and use URDF files specifically for humanoid robots, including their complex kinematic chains and joint configurations.

---

## Introduction to URDF

URDF defines a robot's:

*   **Links**: Physical body segments (torso, head, arms, legs)
*   **Joints**: Connections between links (hips, knees, ankles)
*   **Visual geometry**: What the robot looks like
*   **Collision geometry**: Simplified shapes for physics simulation
*   **Inertial properties**: Mass and moments of inertia

### URDF vs SDF

| Aspect | URDF | SDF (Simulation Description Format) |
|--------|------|-------------------------------------|
| **Origin** | ROS ecosystem | Gazebo simulator |
| **Closed loops** | Not supported | Supported |
| **Multiple robots** | One per file | Multiple per file |
| **Sensors** | Limited | Extensive support |
| **Use case** | ROS robot description | Simulation worlds |

For humanoid robots, URDF is typically used for robot description in ROS, then converted to SDF for Gazebo simulation.

---

## URDF Basic Structure

### Minimal URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base link (pelvis/torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## Joint Types for Humanoids

### Common Joint Types

| Type | DOF | Description | Humanoid Use |
|------|-----|-------------|--------------|
| `revolute` | 1 | Rotation with limits | Hips, knees, ankles |
| `continuous` | 1 | Unlimited rotation | Wheels (if any) |
| `prismatic` | 1 | Linear motion | Grippers |
| `fixed` | 0 | No motion | Sensor mounts |
| `floating` | 6 | Free in space | Base (for simulation) |
| `planar` | 3 | Motion in a plane | Rarely used |

### Revolute Joint Definition

```xml
<joint name="left_hip_pitch" type="revolute">
  <parent link="pelvis"/>
  <child link="left_thigh"/>
  <origin xyz="0.0 0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="0.52"
         velocity="3.0" effort="100.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Joint Properties Explained

*   **origin**: Position and orientation of the joint frame relative to parent
*   **axis**: Rotation axis (for revolute) or translation axis (for prismatic)
*   **limit**: Motion range, velocity, and torque limits
*   **dynamics**: Damping and friction coefficients

---

## Humanoid Kinematic Structure

### Typical Humanoid Link Hierarchy

```text
                           [base_link/pelvis]
                                   │
            ┌──────────────────────┼──────────────────────┐
            │                      │                      │
      [left_hip]              [torso]              [right_hip]
            │                      │                      │
      [left_thigh]            [chest]            [right_thigh]
            │                      │                      │
      [left_shin]     ┌───────────┴───────────┐   [right_shin]
            │         │           │           │         │
      [left_foot] [left_arm]   [head]   [right_arm] [right_foot]
                      │                       │
                [left_hand]            [right_hand]
```

### Degrees of Freedom by Body Part

| Body Part | Typical DOF | Joint Names |
|-----------|-------------|-------------|
| **Hip** | 3 | yaw, roll, pitch |
| **Knee** | 1 | pitch |
| **Ankle** | 2 | pitch, roll |
| **Shoulder** | 3 | pitch, roll, yaw |
| **Elbow** | 1 | pitch |
| **Wrist** | 2-3 | pitch, roll, (yaw) |
| **Neck** | 2-3 | pitch, roll, (yaw) |

---

## Complete Humanoid Leg URDF

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="metal_gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <material name="joint_blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>

  <!-- Pelvis (base) -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.25 0.3 0.15"/>
      </geometry>
      <material name="metal_gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.25 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.05" ixy="0" ixz="0"
               iyy="0.08" iyz="0" izz="0.06"/>
    </inertial>
  </link>

  <!-- ========== LEFT LEG ========== -->

  <!-- Left Hip Yaw -->
  <link name="left_hip_yaw_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.06"/>
      </geometry>
      <material name="joint_blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_yaw" type="revolute">
    <parent link="pelvis"/>
    <child link="left_hip_yaw_link"/>
    <origin xyz="0 0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" velocity="3.0" effort="80.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Left Hip Roll -->
  <link name="left_hip_roll_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.06"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="joint_blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.06"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_roll" type="revolute">
    <parent link="left_hip_yaw_link"/>
    <child link="left_hip_roll_link"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.4" upper="0.4" velocity="3.0" effort="80.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Left Hip Pitch / Thigh -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.35"/>
      </geometry>
      <origin xyz="0 0 -0.175"/>
      <material name="metal_gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.35"/>
      </geometry>
      <origin xyz="0 0 -0.175"/>
    </collision>
    <inertial>
      <mass value="3.5"/>
      <origin xyz="0 0 -0.175"/>
      <inertia ixx="0.04" ixy="0" ixz="0"
               iyy="0.04" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="left_hip_roll_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="0.5" velocity="4.0" effort="120.0"/>
    <dynamics damping="0.8" friction="0.2"/>
  </joint>

  <!-- Left Knee / Shin -->
  <link name="left_shin">
    <visual>
      <geometry>
        <box size="0.06 0.06 0.35"/>
      </geometry>
      <origin xyz="0 0 -0.175"/>
      <material name="metal_gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.06 0.06 0.35"/>
      </geometry>
      <origin xyz="0 0 -0.175"/>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.175"/>
      <inertia ixx="0.03" ixy="0" ixz="0"
               iyy="0.03" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="left_knee_pitch" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" velocity="5.0" effort="100.0"/>
    <dynamics damping="0.6" friction="0.15"/>
  </joint>

  <!-- Left Ankle Pitch -->
  <link name="left_ankle_pitch_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
      <material name="joint_blue"/>
    </visual>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0" ixz="0"
               iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_ankle_pitch" type="revolute">
    <parent link="left_shin"/>
    <child link="left_ankle_pitch_link"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.8" upper="0.8" velocity="4.0" effort="60.0"/>
    <dynamics damping="0.4" friction="0.1"/>
  </joint>

  <!-- Left Foot -->
  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.03"/>
      </geometry>
      <origin xyz="0.05 0 -0.015"/>
      <material name="metal_gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.03"/>
      </geometry>
      <origin xyz="0.05 0 -0.015"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.015"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.003" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="left_ankle_roll" type="revolute">
    <parent link="left_ankle_pitch_link"/>
    <child link="left_foot"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.4" upper="0.4" velocity="3.0" effort="40.0"/>
    <dynamics damping="0.3" friction="0.1"/>
  </joint>

</robot>
```

---

## Using Xacro for Modular URDFs

Xacro (XML Macro) allows parameterized, reusable URDF components.

### Basic Xacro Syntax

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties (variables) -->
  <xacro:property name="thigh_length" value="0.35"/>
  <xacro:property name="shin_length" value="0.35"/>
  <xacro:property name="thigh_mass" value="3.5"/>

  <!-- Macro for leg links -->
  <xacro:macro name="leg_link" params="name length mass">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="0.08 0.08 ${length}"/>
        </geometry>
        <origin xyz="0 0 ${-length/2}"/>
      </visual>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 ${-length/2}"/>
        <inertia ixx="${mass * length * length / 12}"
                 ixy="0" ixz="0"
                 iyy="${mass * length * length / 12}"
                 iyz="0" izz="${mass * 0.08 * 0.08 / 6}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:leg_link name="left_thigh"
                  length="${thigh_length}"
                  mass="${thigh_mass}"/>

  <xacro:leg_link name="left_shin"
                  length="${shin_length}"
                  mass="2.5"/>

</robot>
```

### Complete Leg Macro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="humanoid_leg" params="side parent reflect">

    <!-- Leg attachment point offset -->
    <xacro:property name="hip_offset_y" value="${0.1 * reflect}"/>

    <!-- Hip Yaw Joint -->
    <joint name="${side}_hip_yaw" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_hip_yaw_link"/>
      <origin xyz="0 ${hip_offset_y} -0.05"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.5" upper="0.5" velocity="3.0" effort="80"/>
    </joint>

    <link name="${side}_hip_yaw_link">
      <xacro:default_inertial mass="0.5"/>
    </link>

    <!-- Hip Roll Joint -->
    <joint name="${side}_hip_roll" type="revolute">
      <parent link="${side}_hip_yaw_link"/>
      <child link="${side}_hip_roll_link"/>
      <origin xyz="0 0 -0.05"/>
      <axis xyz="1 0 0"/>
      <limit lower="${-0.4 * reflect}" upper="${0.4 * reflect}"
             velocity="3.0" effort="80"/>
    </joint>

    <link name="${side}_hip_roll_link">
      <xacro:default_inertial mass="0.5"/>
    </link>

    <!-- Hip Pitch Joint + Thigh -->
    <joint name="${side}_hip_pitch" type="revolute">
      <parent link="${side}_hip_roll_link"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 0 -0.05"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.5" upper="0.5" velocity="4.0" effort="120"/>
    </joint>

    <xacro:leg_link name="${side}_thigh"
                    length="${thigh_length}"
                    mass="${thigh_mass}"/>

    <!-- Continue with knee, ankle, foot... -->

  </xacro:macro>

  <!-- Create both legs -->
  <xacro:humanoid_leg side="left" parent="pelvis" reflect="1"/>
  <xacro:humanoid_leg side="right" parent="pelvis" reflect="-1"/>

</robot>
```

---

## Processing URDF Files

### Converting Xacro to URDF

```bash
# Install xacro
sudo apt install ros-humble-xacro

# Convert xacro to URDF
xacro humanoid.urdf.xacro > humanoid.urdf

# With parameters
xacro humanoid.urdf.xacro leg_length:=0.4 > humanoid.urdf
```

### Validating URDF

```bash
# Check URDF syntax
check_urdf humanoid.urdf

# Visualize link tree
urdf_to_graphviz humanoid.urdf
evince humanoid.pdf
```

### Viewing in RViz2

```bash
# Install robot state publisher
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui

# Launch with URDF
ros2 launch urdf_tutorial display.launch.py model:=/path/to/humanoid.urdf
```

---

## Integrating URDF with ROS 2

### Robot State Publisher

```python
# launch/display_humanoid.launch.py

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'humanoid.rviz')]
        )
    ])
```

### Package Structure for Robot Description

```text
humanoid_description/
├── urdf/
│   ├── humanoid.urdf.xacro
│   ├── leg.urdf.xacro
│   ├── arm.urdf.xacro
│   └── sensors.urdf.xacro
├── meshes/
│   ├── visual/
│   │   ├── torso.dae
│   │   ├── thigh.dae
│   │   └── ...
│   └── collision/
│       ├── torso.stl
│       └── ...
├── launch/
│   └── display.launch.py
├── rviz/
│   └── humanoid.rviz
├── config/
│   └── joint_limits.yaml
├── package.xml
└── CMakeLists.txt
```

---

## Adding Sensors to URDF

### Camera Sensor

```xml
<link name="head_camera">
  <visual>
    <geometry>
      <box size="0.03 0.08 0.02"/>
    </geometry>
  </visual>
</link>

<joint name="head_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="head_camera"/>
  <origin xyz="0.06 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo camera plugin (in separate gazebo.urdf.xacro) -->
<gazebo reference="head_camera">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=camera/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </visual>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="pelvis"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>1</always_on>
    <update_rate>200</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## Summary

This chapter covered URDF for humanoid robots:

*   URDF structure: links, joints, visual, collision, and inertial properties
*   Joint types and their configuration for humanoid kinematics
*   Using Xacro for modular, parameterized robot descriptions
*   Processing and validating URDF files
*   Integrating URDF with ROS 2 via robot_state_publisher
*   Adding sensors for simulation

In the next chapter, we will explore how to integrate AI agents with ROS 2 to create intelligent humanoid behaviors.
