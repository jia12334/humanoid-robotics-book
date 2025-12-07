# Chapter 11: Sensors Simulation - LiDAR, Depth, IMU

Accurate sensor simulation is essential for developing perception algorithms that transfer to real robots. This chapter covers configuring and using simulated sensors in Gazebo for humanoid robotics.

---

## Sensor Types Overview

| Sensor | Data Type | Use Case | Update Rate |
|--------|-----------|----------|-------------|
| **Camera** | RGB Image | Object detection, navigation | 30-60 Hz |
| **Depth Camera** | Depth + RGB | 3D perception, obstacle avoidance | 30 Hz |
| **LiDAR** | Point Cloud | Mapping, localization | 10-20 Hz |
| **IMU** | Orientation, acceleration | Balance, pose estimation | 100-400 Hz |
| **Force-Torque** | 6-axis wrench | Contact detection, manipulation | 100-1000 Hz |
| **Joint Encoder** | Position, velocity | Motor feedback | 1000 Hz |

---

## Camera Sensors

### RGB Camera Configuration

```xml
<link name="head_camera_link">
  <pose relative_to="head">0.05 0 0.05 0 0 0</pose>

  <sensor name="head_camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>30</update_rate>

    <camera name="head_cam">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </sensor>
</link>
```

### Camera Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `horizontal_fov` | Field of view (radians) | 1.047 (60°) to 1.57 (90°) |
| `width` × `height` | Resolution | 640×480, 1280×720 |
| `format` | Pixel format | R8G8B8, L8 (grayscale) |
| `clip/near` | Min visible distance | 0.05 - 0.1 m |
| `clip/far` | Max visible distance | 10 - 100 m |

---

## Depth Cameras

### Intel RealSense Style Sensor

```xml
<link name="depth_camera_link">
  <pose relative_to="torso">0.3 0 0.2 0 0 0</pose>

  <sensor name="rgbd_camera" type="rgbd_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>

    <camera name="rgbd_cam">
      <horizontal_fov>1.21</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <depth_camera>
        <clip>
          <near>0.3</near>
          <far>10.0</far>
        </clip>
      </depth_camera>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </camera>

    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </sensor>
</link>
```

### ROS 2 Bridge for Depth Camera

```yaml
# depth_bridge.yaml
- ros_topic_name: /camera/color/image_raw
  gz_topic_name: /rgbd_camera/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: ignition.msgs.Image
  direction: GZ_TO_ROS

- ros_topic_name: /camera/depth/image_raw
  gz_topic_name: /rgbd_camera/depth_image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: ignition.msgs.Image
  direction: GZ_TO_ROS

- ros_topic_name: /camera/camera_info
  gz_topic_name: /rgbd_camera/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: ignition.msgs.CameraInfo
  direction: GZ_TO_ROS

- ros_topic_name: /camera/points
  gz_topic_name: /rgbd_camera/points
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: ignition.msgs.PointCloudPacked
  direction: GZ_TO_ROS
```

---

## LiDAR Sensors

### 2D LiDAR (RPLidar Style)

```xml
<link name="lidar_link">
  <pose relative_to="base_link">0 0 0.5 0 0 0</pose>

  <sensor name="lidar_2d" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>true</visualize>

    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>1</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </lidar>

    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </sensor>
</link>
```

### 3D LiDAR (Velodyne Style)

```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>

  <lidar>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>
        <max_angle>0.2618</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.5</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
</sensor>
```

### LiDAR ROS 2 Bridge

```python
# In launch file
bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
    ],
    output='screen'
)
```

---

## IMU Sensors

### IMU Configuration

```xml
<link name="imu_link">
  <pose relative_to="base_link">0 0 0.1 0 0 0</pose>

  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin
      filename="libignition-gazebo-imu-system.so"
      name="ignition::gazebo::systems::Imu">
    </plugin>
  </sensor>
</link>
```

### IMU Noise Models

Realistic IMU simulation includes several noise types:

| Noise Type | Description | Typical Value |
|------------|-------------|---------------|
| **White noise** | Random Gaussian | σ = 0.001-0.01 rad/s |
| **Bias** | Constant offset | 0.0001-0.001 rad/s |
| **Random walk** | Drift over time | Model-specific |
| **Scale factor** | Gain error | 0.1-1% |

### IMU Processing Node

```python
#!/usr/bin/env python3
# imu_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from scipy.spatial.transform import Rotation
import numpy as np

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.euler_pub = self.create_publisher(
            Vector3Stamped, '/imu/euler', 10)

        # Complementary filter state
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.alpha = 0.98  # Gyro weight

        self.last_time = None

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9

            # Get gyroscope data
            gyro = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            # Get accelerometer data
            accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            # Integrate gyroscope
            gyro_quat = self.gyro_to_quaternion(gyro, dt)
            gyro_orientation = self.quaternion_multiply(
                self.orientation, gyro_quat)

            # Calculate orientation from accelerometer
            accel_orientation = self.accel_to_quaternion(accel)

            # Complementary filter
            self.orientation = self.slerp(
                accel_orientation, gyro_orientation, self.alpha)
            self.orientation /= np.linalg.norm(self.orientation)

            # Publish euler angles
            euler = Rotation.from_quat(
                [self.orientation[1], self.orientation[2],
                 self.orientation[3], self.orientation[0]]
            ).as_euler('xyz')

            euler_msg = Vector3Stamped()
            euler_msg.header = msg.header
            euler_msg.vector.x = euler[0]
            euler_msg.vector.y = euler[1]
            euler_msg.vector.z = euler[2]
            self.euler_pub.publish(euler_msg)

        self.last_time = current_time

    def gyro_to_quaternion(self, gyro, dt):
        angle = np.linalg.norm(gyro) * dt
        if angle > 1e-10:
            axis = gyro / np.linalg.norm(gyro)
            return np.array([
                np.cos(angle / 2),
                axis[0] * np.sin(angle / 2),
                axis[1] * np.sin(angle / 2),
                axis[2] * np.sin(angle / 2)
            ])
        return np.array([1.0, 0.0, 0.0, 0.0])

    def accel_to_quaternion(self, accel):
        # Simplified - assumes gravity is the only acceleration
        accel_norm = accel / np.linalg.norm(accel)
        pitch = np.arcsin(-accel_norm[0])
        roll = np.arctan2(accel_norm[1], accel_norm[2])
        return Rotation.from_euler('xyz', [roll, pitch, 0]).as_quat()

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def slerp(self, q1, q2, t):
        # Simplified slerp
        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
            return q1 + t * (q2 - q1)
        theta = np.arccos(dot)
        return (np.sin((1-t)*theta)*q1 + np.sin(t*theta)*q2) / np.sin(theta)

def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Force-Torque Sensors

### Joint Force-Torque Sensor

```xml
<joint name="wrist_joint" type="revolute">
  <parent>forearm</parent>
  <child>hand</child>
  <!-- Joint configuration... -->

  <sensor name="wrist_ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>

    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</joint>
```

### Contact Sensors

```xml
<link name="fingertip">
  <collision name="fingertip_collision">
    <geometry>
      <sphere><radius>0.01</radius></sphere>
    </geometry>
  </collision>

  <sensor name="fingertip_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>100</update_rate>

    <contact>
      <collision>fingertip_collision</collision>
    </contact>
  </sensor>
</link>
```

---

## Complete Sensor Launch

```python
# launch/sensors_bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Gazebo-ROS bridge for all sensors
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Camera
                '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                # Depth
                '/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # LiDAR
                '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # IMU
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            output='screen'
        ),

        # IMU processor
        Node(
            package='humanoid_perception',
            executable='imu_processor',
            name='imu_processor',
            output='screen'
        ),
    ])
```

---

## Summary

This chapter covered sensor simulation in Gazebo:

*   **Cameras**: RGB and depth camera configuration
*   **LiDAR**: 2D and 3D laser scanners
*   **IMU**: Inertial measurement with noise models
*   **Force-Torque**: Contact and wrench sensing
*   **ROS 2 Bridge**: Connecting simulated sensors to robot software

Accurate sensor simulation enables development of perception algorithms that work on real hardware. In the next chapter, we will explore Unity for enhanced visualization.
