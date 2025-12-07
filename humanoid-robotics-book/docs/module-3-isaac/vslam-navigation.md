# Chapter 16: Isaac ROS - VSLAM and Navigation

Isaac ROS provides GPU-accelerated perception and navigation packages optimized for NVIDIA hardware. This chapter covers Visual SLAM and navigation capabilities for humanoid robots.

---

## Isaac ROS Overview

Isaac ROS packages provide:

*   **GPU acceleration**: CUDA-optimized algorithms
*   **DNN integration**: Deep learning perception pipelines
*   **Jetson optimization**: Efficient edge deployment
*   **ROS 2 native**: Standard interfaces and tools

### Key Packages

| Package | Function |
|---------|----------|
| `isaac_ros_visual_slam` | Visual SLAM with stereo/depth |
| `isaac_ros_nvblox` | 3D reconstruction and mapping |
| `isaac_ros_dnn_inference` | GPU-accelerated inference |
| `isaac_ros_apriltag` | Fiducial marker detection |
| `isaac_ros_depth_segmentation` | Depth-based segmentation |

---

## Installation

### Prerequisites

```bash
# Install Isaac ROS common
sudo apt install ros-humble-isaac-ros-common

# Install Visual SLAM
sudo apt install ros-humble-isaac-ros-visual-slam

# Install Nvblox
sudo apt install ros-humble-isaac-ros-nvblox
```

### Docker Setup (Recommended)

```bash
# Pull Isaac ROS container
docker pull nvcr.io/nvidia/isaac/ros:humble-nav2_x86_64

# Run container
docker run --runtime nvidia -it --rm \
  --network host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  nvcr.io/nvidia/isaac/ros:humble-nav2_x86_64
```

---

## Visual SLAM (cuVSLAM)

### Architecture

```text
┌─────────────┐     ┌─────────────────────────────────────┐
│   Stereo    │────▶│         Isaac ROS Visual SLAM       │
│   Camera    │     │  ┌──────────┐  ┌────────────────┐  │
└─────────────┘     │  │ Feature  │  │   Pose Graph   │  │
                    │  │ Tracking │──│   Optimization │  │
┌─────────────┐     │  └──────────┘  └────────────────┘  │
│    IMU      │────▶│                                     │
└─────────────┘     └──────────────────┬──────────────────┘
                                       │
                    ┌──────────────────▼──────────────────┐
                    │  /visual_slam/tracking/odometry     │
                    │  /visual_slam/vis/landmarks_cloud   │
                    │  /tf (base_link → odom)             │
                    └─────────────────────────────────────┘
```

### Launch Configuration

```python
# launch/visual_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    visual_slam_node = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam',
        remappings=[
            ('stereo_camera/left/image', '/camera/left/image_raw'),
            ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
            ('stereo_camera/right/image', '/camera/right/image_raw'),
            ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
            ('visual_slam/imu', '/imu/data'),
        ],
        parameters=[{
            'denoise_input_images': True,
            'rectified_images': False,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'img_jitter_threshold_ms': 34.0,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )

    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
```

### Using Visual SLAM Output

```python
#!/usr/bin/env python3
# vslam_pose_subscriber.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class VSLAMPoseSubscriber(Node):
    def __init__(self):
        super().__init__('vslam_pose_subscriber')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )

    def odom_callback(self, msg: Odometry):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # Log position
        pos = msg.pose.pose.position
        self.get_logger().info(
            f'Robot position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
        )
```

---

## Nvblox 3D Mapping

### Overview

Nvblox creates real-time 3D reconstructions for navigation:

*   **TSDF mapping**: Truncated Signed Distance Fields
*   **ESDF generation**: Euclidean Signed Distance Fields for planning
*   **GPU acceleration**: Real-time performance on Jetson

### Launch Configuration

```python
# launch/nvblox_mapping.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        parameters=[{
            # Input configuration
            'global_frame': 'odom',
            'pose_frame': 'base_link',

            # Mapping parameters
            'voxel_size': 0.05,
            'tsdf_integrator_max_integration_distance_m': 5.0,

            # Mesh output
            'mesh_update_rate_hz': 5.0,

            # ESDF for planning
            'esdf_update_rate_hz': 10.0,
            'esdf_2d_min_height': 0.0,
            'esdf_2d_max_height': 1.0,

            # Sensor configuration
            'use_depth': True,
            'use_lidar': False,
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('color/image', '/camera/color/image_raw'),
            ('color/camera_info', '/camera/color/camera_info'),
        ]
    )

    return LaunchDescription([nvblox_node])
```

### Costmap Integration

```python
# Nvblox to Nav2 costmap integration
nvblox_costmap_node = Node(
    package='nvblox_ros',
    executable='nvblox_costmap_2d_node',
    parameters=[{
        'costmap_topic': '/nvblox_costmap',
        'esdf_slice_height': 0.5,
        'costmap_resolution': 0.05,
    }]
)
```

---

## Sensor Fusion

### IMU + Visual Odometry

```yaml
# visual_slam_params.yaml
visual_slam:
  ros__parameters:
    enable_imu_fusion: true

    # IMU noise parameters (from datasheet)
    gyro_noise_density: 0.000244      # rad/s/sqrt(Hz)
    gyro_random_walk: 0.000019393     # rad/s^2/sqrt(Hz)
    accel_noise_density: 0.001862     # m/s^2/sqrt(Hz)
    accel_random_walk: 0.003          # m/s^3/sqrt(Hz)

    # Calibration
    imu_from_left_camera:
      - [0.999, 0.0, 0.0, 0.05]
      - [0.0, 0.999, 0.0, 0.0]
      - [0.0, 0.0, 0.999, 0.02]
      - [0.0, 0.0, 0.0, 1.0]
```

### Multi-Sensor Fusion Node

```python
#!/usr/bin/env python3
# sensor_fusion_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from robot_localization import ExtendedKalmanFilter

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribers
        self.vslam_sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry',
            self.vslam_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/wheel_odometry', self.wheel_callback, 10)

        # Publisher
        self.fused_odom_pub = self.create_publisher(
            Odometry, '/odometry/filtered', 10)

        # EKF state
        self.ekf = ExtendedKalmanFilter()

    def vslam_callback(self, msg):
        self.ekf.update_visual(msg)
        self.publish_fused()

    def imu_callback(self, msg):
        self.ekf.predict_imu(msg)

    def wheel_callback(self, msg):
        self.ekf.update_wheel(msg)
        self.publish_fused()

    def publish_fused(self):
        fused_msg = self.ekf.get_state_as_odometry()
        self.fused_odom_pub.publish(fused_msg)
```

---

## Localization Modes

### Mapping Mode

```bash
# Create new map while navigating
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  enable_slam_visualization:=true \
  enable_localization_n_mapping:=true
```

### Localization Only

```bash
# Localize in existing map
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py \
  enable_localization_n_mapping:=false \
  map_frame:=map \
  load_map_path:=/maps/building_map.db
```

### Map Saving

```python
# Save visual SLAM map
from isaac_ros_visual_slam.srv import SaveMap

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.save_client = self.create_client(
            SaveMap, '/visual_slam/save_map')

    def save_map(self, path):
        request = SaveMap.Request()
        request.map_url = path
        future = self.save_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

---

## Performance Optimization

### Jetson Configuration

```bash
# Enable maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor GPU usage
tegrastats
```

### Launch Parameters for Jetson

```yaml
# jetson_vslam_params.yaml
visual_slam:
  ros__parameters:
    # Reduce resolution for Jetson
    input_img_width: 640
    input_img_height: 480

    # Reduce feature count
    max_features_per_frame: 500

    # GPU memory optimization
    enable_ground_constraint_in_odometry: true
    enable_localization_n_mapping: true
```

---

## Summary

This chapter covered Isaac ROS VSLAM and navigation:

*   **Visual SLAM**: GPU-accelerated localization and mapping
*   **Nvblox**: Real-time 3D reconstruction
*   **Sensor fusion**: Combining visual, inertial, and wheel odometry
*   **Localization modes**: Mapping vs localization-only
*   **Performance**: Jetson optimization

Isaac ROS enables robust navigation for humanoid robots. In the next chapter, we will explore Nav2 path planning integration.
