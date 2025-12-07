# Chapter 10: Gazebo Physics & Collisions

Gazebo is a powerful robotics simulator that provides realistic physics simulation for testing humanoid robots. This chapter covers physics engines, collision detection, and environment setup for digital twin development.

---

## Introduction to Gazebo

Gazebo (now Gazebo Sim, formerly Ignition Gazebo) provides:

*   **Physics Simulation**: Realistic rigid body dynamics
*   **Sensor Simulation**: Cameras, LiDAR, IMU, contact sensors
*   **ROS 2 Integration**: Direct communication with robot software
*   **Visualization**: 3D rendering of robots and environments

### Gazebo Versions

| Version | Status | ROS 2 Compatibility |
|---------|--------|---------------------|
| Gazebo Garden | Latest LTS | Humble, Iron |
| Gazebo Fortress | LTS | Humble |
| Gazebo Classic (11) | Legacy | Foxy, Galactic |

---

## Physics Engine Overview

### Available Physics Engines

Gazebo supports multiple physics backends:

| Engine | Strengths | Use Case |
|--------|-----------|----------|
| **DART** | Accurate joint dynamics | Humanoid simulation (default) |
| **Bullet** | Fast collision detection | Large environments |
| **ODE** | Legacy compatibility | Older simulations |
| **TPE** | Trivial physics | UI development, rapid testing |

### Configuring Physics

```xml
<!-- world.sdf -->
<world name="humanoid_world">
  <physics name="dart_physics" type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>

    <dart>
      <collision_detector>bullet</collision_detector>
      <solver>
        <solver_type>dantzig</solver_type>
      </solver>
    </dart>
  </physics>
</world>
```

### Physics Parameters Explained

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `max_step_size` | Simulation timestep (seconds) | 0.001 for humanoids |
| `real_time_factor` | Target simulation speed | 1.0 = real-time |
| `real_time_update_rate` | Physics updates per second | 1000 Hz |

---

## Rigid Body Dynamics

### Inertial Properties

Accurate mass and inertia are critical for humanoid balance:

```xml
<link name="torso">
  <inertial>
    <mass>15.0</mass>
    <pose>0 0 0.1 0 0 0</pose>
    <inertia>
      <ixx>0.5</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.4</iyy>
      <iyz>0</iyz>
      <izz>0.3</izz>
    </inertia>
  </inertial>
</link>
```

### Calculating Inertia

For common shapes:

**Box (w × h × d, mass m):**
```
Ixx = m/12 * (h² + d²)
Iyy = m/12 * (w² + d²)
Izz = m/12 * (w² + h²)
```

**Cylinder (radius r, length l, mass m):**
```
Ixx = Iyy = m/12 * (3r² + l²)
Izz = m/2 * r²
```

**Sphere (radius r, mass m):**
```
Ixx = Iyy = Izz = 2/5 * m * r²
```

---

## Collision Detection

### Collision Geometry

Collision shapes should be simplified for performance:

```xml
<link name="thigh">
  <!-- Visual: detailed mesh -->
  <visual name="thigh_visual">
    <geometry>
      <mesh>
        <uri>meshes/thigh_detailed.dae</uri>
      </mesh>
    </geometry>
  </visual>

  <!-- Collision: simplified shape -->
  <collision name="thigh_collision">
    <geometry>
      <box>
        <size>0.1 0.1 0.35</size>
      </box>
    </geometry>
  </collision>
</link>
```

### Collision Shapes Comparison

| Shape | Performance | Accuracy | Use Case |
|-------|-------------|----------|----------|
| Box | Fastest | Low | Torso, simple links |
| Cylinder | Fast | Medium | Arms, legs |
| Sphere | Fast | Low | Joints, wheels |
| Capsule | Fast | Medium | Limbs |
| Mesh | Slow | High | Complex grippers |
| Convex Hull | Medium | High | Simplified meshes |

### Surface Properties

Control friction and contact behavior:

```xml
<collision name="foot_collision">
  <geometry>
    <box><size>0.2 0.1 0.03</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
        <fdir1>0 0 1</fdir1>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e6</kp>
        <kd>100</kd>
        <max_vel>0.1</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

### Surface Parameters

| Parameter | Description | Humanoid Value |
|-----------|-------------|----------------|
| `mu` / `mu2` | Friction coefficients | 0.8-1.2 for feet |
| `kp` | Contact stiffness | 1e5 - 1e7 |
| `kd` | Contact damping | 10 - 1000 |
| `max_vel` | Max contact correction velocity | 0.01 - 0.1 |
| `min_depth` | Penetration threshold | 0.001 |

---

## Joint Configuration

### Joint Types for Humanoids

```xml
<!-- Revolute joint (most common) -->
<joint name="left_knee" type="revolute">
  <parent>left_thigh</parent>
  <child>left_shin</child>
  <pose relative_to="left_thigh">0 0 -0.35 0 0 0</pose>
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
      <spring_stiffness>0</spring_stiffness>
    </dynamics>
  </axis>
</joint>

<!-- Ball joint (3 DOF) -->
<joint name="shoulder" type="ball">
  <parent>torso</parent>
  <child>upper_arm</child>
  <pose>0.15 0 0.3 0 0 0</pose>
</joint>
```

### Joint Dynamics

| Parameter | Effect | Typical Range |
|-----------|--------|---------------|
| `damping` | Velocity-dependent resistance | 0.1 - 5.0 Nm·s/rad |
| `friction` | Static friction | 0.0 - 1.0 Nm |
| `spring_stiffness` | Joint spring | 0 (usually) |
| `spring_reference` | Spring equilibrium | Joint angle |

---

## Building a Simulation World

### Basic World Structure

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_lab">

    <!-- Physics configuration -->
    <physics name="physics_config" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

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
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
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
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

  </world>
</sdf>
```

### Adding Objects

```xml
<!-- Table for manipulation tasks -->
<model name="table">
  <static>true</static>
  <pose>1.5 0 0 0 0 0</pose>
  <link name="link">
    <collision name="surface">
      <pose>0 0 0.75 0 0 0</pose>
      <geometry>
        <box><size>1.0 0.6 0.05</size></box>
      </geometry>
    </collision>
    <visual name="surface_visual">
      <pose>0 0 0.75 0 0 0</pose>
      <geometry>
        <box><size>1.0 0.6 0.05</size></box>
      </geometry>
      <material>
        <ambient>0.6 0.4 0.2 1</ambient>
      </material>
    </visual>
    <!-- Table legs -->
    <collision name="leg1">
      <pose>0.45 0.25 0.375 0 0 0</pose>
      <geometry>
        <box><size>0.05 0.05 0.75</size></box>
      </geometry>
    </collision>
    <!-- Additional legs... -->
  </link>
</model>

<!-- Graspable object -->
<model name="cup">
  <pose>1.5 0 0.8 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>0.3</mass>
      <inertia>
        <ixx>0.0005</ixx>
        <iyy>0.0005</iyy>
        <izz>0.0003</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.04</radius>
          <length>0.1</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.04</radius>
          <length>0.1</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

---

## ROS 2 Integration

### Gazebo ROS Packages

```bash
# Install Gazebo ROS packages
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge

# Launch Gazebo with ROS bridge
ros2 launch ros_gz_sim gz_sim.launch.py world_sdf_file:=world.sdf
```

### ros_gz_bridge Configuration

```yaml
# bridge_config.yaml
- ros_topic_name: /joint_states
  gz_topic_name: /world/humanoid_world/model/humanoid/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: ignition.msgs.Model
  direction: GZ_TO_ROS

- ros_topic_name: /cmd_vel
  gz_topic_name: /model/humanoid/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: ignition.msgs.Twist
  direction: ROS_TO_GZ

- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: ignition.msgs.Clock
  direction: GZ_TO_ROS
```

### Launch File with Bridge

```python
# launch/gazebo_humanoid.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'humanoid_lab.sdf')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid',
            '-file', os.path.join(pkg_share, 'models', 'humanoid.sdf'),
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge
    ])
```

---

## Contact and Force Sensing

### Contact Sensor Plugin

```xml
<link name="left_foot">
  <!-- Collision and visual... -->

  <sensor name="foot_contact" type="contact">
    <contact>
      <collision>foot_collision</collision>
    </contact>
    <update_rate>100</update_rate>
    <plugin
      filename="libignition-gazebo-contact-system.so"
      name="ignition::gazebo::systems::Contact">
    </plugin>
  </sensor>
</link>
```

### Force-Torque Sensor

```xml
<joint name="left_ankle" type="revolute">
  <parent>left_shin</parent>
  <child>left_foot</child>
  <!-- Joint configuration... -->

  <sensor name="ankle_ft" type="force_torque">
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</joint>
```

---

## Debugging Physics Issues

### Common Problems and Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot explodes on spawn | Interpenetrating collisions | Spawn higher, check collision geometry |
| Jittery motion | Timestep too large | Reduce `max_step_size` |
| Falls through floor | Missing collision | Add collision geometry |
| Slow simulation | Complex meshes | Use primitive collision shapes |
| Unstable joints | High gains, low damping | Tune PID, increase damping |

### Visualization Aids

```bash
# Enable collision visualization
gz sim -r world.sdf --render-engine ogre2

# In Gazebo GUI: View -> Collisions -> Show
```

---

## Summary

This chapter covered Gazebo physics simulation:

*   **Physics Engines**: DART for humanoid dynamics
*   **Rigid Body Dynamics**: Mass, inertia, and their importance
*   **Collision Detection**: Simplified geometry, surface properties
*   **Joint Configuration**: Types, limits, and dynamics
*   **World Building**: Environments for testing
*   **ROS 2 Integration**: Bridge configuration and launch files

In the next chapter, we will add sensors to our simulation for perception testing.
