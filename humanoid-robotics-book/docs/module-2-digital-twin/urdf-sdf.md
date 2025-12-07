# Chapter 13: URDF/SDF Models in Simulation

This chapter covers the two primary robot description formats used in simulation: URDF (Unified Robot Description Format) and SDF (Simulation Description Format). Understanding both formats is essential for effective digital twin development.

---

## URDF vs SDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| **Origin** | ROS | Gazebo/SDFormat |
| **Primary use** | Robot description | World + robot description |
| **Closed loops** | Not supported | Supported |
| **Multiple robots** | One per file | Multiple per file |
| **Sensors** | Via Gazebo plugins | Native support |
| **Environments** | Not supported | Full world description |
| **Extensibility** | Limited | Highly extensible |

### When to Use Each

*   **URDF**: ROS robot description, RViz visualization, MoveIt configuration
*   **SDF**: Gazebo simulation, complex environments, multi-robot scenarios

---

## URDF Structure Review

### Complete Humanoid Arm Example

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="aluminum">
    <color rgba="0.8 0.8 0.8 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="aluminum"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder Link -->
  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="aluminum"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Upper Arm -->
  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.3"/>
      </geometry>
      <material name="aluminum"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.015" ixy="0" ixz="0"
               iyy="0.015" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="shoulder_roll_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="40" velocity="2"/>
  </joint>

  <!-- Elbow Joint and Forearm -->
  <link name="forearm_link">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.25"/>
      </geometry>
      <material name="aluminum"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.125"/>
      <inertia ixx="0.006" ixy="0" ixz="0"
               iyy="0.006" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="30" velocity="3"/>
  </joint>

  <!-- Gazebo-specific configurations -->
  <gazebo reference="shoulder_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_arm</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

---

## SDF Structure

### World File Structure

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_world">

    <!-- Physics configuration -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene configuration -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include robot model -->
    <include>
      <uri>model://humanoid</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>

    <!-- Plugins -->
    <plugin
      filename="libignition-gazebo-physics-system.so"
      name="ignition::gazebo::systems::Physics">
    </plugin>

    <plugin
      filename="libignition-gazebo-scene-broadcaster-system.so"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

  </world>
</sdf>
```

### SDF Model Structure

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="humanoid_leg">
    <pose>0 0 0 0 0 0</pose>

    <!-- Thigh Link -->
    <link name="thigh">
      <pose>0 0 0 0 0 0</pose>

      <inertial>
        <mass>3.5</mass>
        <pose>0 0 -0.175 0 0 0</pose>
        <inertia>
          <ixx>0.04</ixx>
          <iyy>0.04</iyy>
          <izz>0.005</izz>
        </inertia>
      </inertial>

      <visual name="thigh_visual">
        <pose>0 0 -0.175 0 0 0</pose>
        <geometry>
          <box>
            <size>0.08 0.08 0.35</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>

      <collision name="thigh_collision">
        <pose>0 0 -0.175 0 0 0</pose>
        <geometry>
          <box>
            <size>0.08 0.08 0.35</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Shin Link -->
    <link name="shin">
      <pose relative_to="knee_joint">0 0 0 0 0 0</pose>

      <inertial>
        <mass>2.5</mass>
        <pose>0 0 -0.175 0 0 0</pose>
        <inertia>
          <ixx>0.03</ixx>
          <iyy>0.03</iyy>
          <izz>0.003</izz>
        </inertia>
      </inertial>

      <visual name="shin_visual">
        <pose>0 0 -0.175 0 0 0</pose>
        <geometry>
          <box>
            <size>0.06 0.06 0.35</size>
          </box>
        </geometry>
      </visual>

      <collision name="shin_collision">
        <pose>0 0 -0.175 0 0 0</pose>
        <geometry>
          <box>
            <size>0.06 0.06 0.35</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Knee Joint -->
    <joint name="knee_joint" type="revolute">
      <parent>thigh</parent>
      <child>shin</child>
      <pose relative_to="thigh">0 0 -0.35 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>0</lower>
          <upper>2.5</upper>
          <effort>100</effort>
          <velocity>5</velocity>
        </limit>
        <dynamics>
          <damping>0.5</damping>
          <friction>0.1</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- Sensors -->
    <link name="imu_link">
      <pose relative_to="thigh">0 0 0 0 0 0</pose>
      <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
      </sensor>
    </link>

    <joint name="imu_joint" type="fixed">
      <parent>thigh</parent>
      <child>imu_link</child>
    </joint>

  </model>
</sdf>
```

---

## Converting Between Formats

### URDF to SDF Conversion

```bash
# Using gz sdf tool
gz sdf -p robot.urdf > robot.sdf

# Or using the Python API
from urdf_parser_py.urdf import URDF
# Note: Full conversion requires additional processing
```

### Key Conversion Considerations

| URDF Element | SDF Equivalent | Notes |
|--------------|----------------|-------|
| `<robot>` | `<model>` | Name preserved |
| `<link>` | `<link>` | Direct mapping |
| `<joint>` | `<joint>` | Syntax differs |
| `<gazebo>` | Embedded directly | No wrapper needed |
| `<transmission>` | Plugins | Different approach |

### Handling URDF Limitations

```xml
<!-- SDF supports features URDF cannot express -->

<!-- Closed kinematic loops -->
<joint name="parallel_link" type="revolute">
  <parent>link_a</parent>
  <child>link_c</child>
  <!-- Creates a loop: base → link_a → link_b → link_c → base -->
</joint>

<!-- Nested models -->
<model name="robot_with_tool">
  <include>
    <uri>model://humanoid_arm</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>
  <include>
    <uri>model://gripper</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
  <joint name="tool_attach" type="fixed">
    <parent>humanoid_arm::wrist</parent>
    <child>gripper::base</child>
  </joint>
</model>
```

---

## Xacro for URDF Generation

### Parameterized Robot Description

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="humanoid">

  <!-- Parameters -->
  <xacro:property name="thigh_length" value="0.35"/>
  <xacro:property name="shin_length" value="0.35"/>
  <xacro:property name="thigh_mass" value="3.5"/>
  <xacro:property name="shin_mass" value="2.5"/>

  <!-- Inertia calculation macro -->
  <xacro:macro name="box_inertia" params="m x y z">
    <inertia ixx="${m*(y*y+z*z)/12}" ixy="0" ixz="0"
             iyy="${m*(x*x+z*z)/12}" iyz="0"
             izz="${m*(x*x+y*y)/12}"/>
  </xacro:macro>

  <!-- Leg macro -->
  <xacro:macro name="leg" params="side reflect">
    <link name="${side}_thigh">
      <visual>
        <origin xyz="0 0 ${-thigh_length/2}"/>
        <geometry>
          <box size="0.08 0.08 ${thigh_length}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 ${-thigh_length/2}"/>
        <geometry>
          <box size="0.08 0.08 ${thigh_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${thigh_mass}"/>
        <origin xyz="0 0 ${-thigh_length/2}"/>
        <xacro:box_inertia m="${thigh_mass}" x="0.08" y="0.08" z="${thigh_length}"/>
      </inertial>
    </link>

    <joint name="${side}_hip" type="revolute">
      <parent link="pelvis"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 ${reflect * 0.1} 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.5" upper="0.5" effort="100" velocity="3"/>
    </joint>

    <link name="${side}_shin">
      <visual>
        <origin xyz="0 0 ${-shin_length/2}"/>
        <geometry>
          <box size="0.06 0.06 ${shin_length}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${shin_mass}"/>
        <origin xyz="0 0 ${-shin_length/2}"/>
        <xacro:box_inertia m="${shin_mass}" x="0.06" y="0.06" z="${shin_length}"/>
      </inertial>
    </link>

    <joint name="${side}_knee" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 ${-thigh_length}"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.5" effort="80" velocity="4"/>
    </joint>
  </xacro:macro>

  <!-- Pelvis -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.25 0.3 0.15"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="8.0"/>
      <xacro:box_inertia m="8.0" x="0.25" y="0.3" z="0.15"/>
    </inertial>
  </link>

  <!-- Create both legs -->
  <xacro:leg side="left" reflect="1"/>
  <xacro:leg side="right" reflect="-1"/>

</robot>
```

### Processing Xacro

```bash
# Generate URDF from Xacro
xacro humanoid.urdf.xacro > humanoid.urdf

# With command-line parameters
xacro humanoid.urdf.xacro thigh_length:=0.4 > humanoid_tall.urdf
```

---

## Model Database Organization

### Gazebo Model Structure

```text
~/.gazebo/models/humanoid/
├── model.config
├── model.sdf
├── meshes/
│   ├── torso.dae
│   ├── thigh.dae
│   └── ...
└── materials/
    └── textures/
        └── metal.png
```

### model.config

```xml
<?xml version="1.0"?>
<model>
  <name>Humanoid Robot</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>

  <author>
    <name>Robotics Team</name>
    <email>team@example.com</email>
  </author>

  <description>
    A bipedal humanoid robot for simulation testing.
  </description>
</model>
```

---

## Spawning Models in Simulation

### ROS 2 Spawn Service

```python
# launch/spawn_humanoid.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_description')
    sdf_file = os.path.join(pkg_share, 'models', 'humanoid', 'model.sdf')

    # Read SDF file
    with open(sdf_file, 'r') as f:
        sdf_content = f.read()

    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'humanoid',
                '-string', sdf_content,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '1.0',
            ],
            output='screen'
        )
    ])
```

---

## Summary

This chapter covered robot description formats for simulation:

*   **URDF**: ROS-native format for robot description
*   **SDF**: Gazebo-native format with advanced features
*   **Conversion**: Moving between formats and handling limitations
*   **Xacro**: Parameterized URDF generation
*   **Model Organization**: File structure and spawning

Understanding both formats enables effective digital twin development across different simulation platforms. In the next module, we will explore NVIDIA Isaac Sim for advanced AI-powered simulation.
