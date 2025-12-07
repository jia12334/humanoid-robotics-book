# Chapter 7: Python rclpy Packages

This chapter guides you through creating complete ROS 2 Python packages using the `rclpy` client library. You will learn how to structure packages, manage dependencies, configure builds, and deploy reusable robotics components for humanoid systems.

---

## Introduction to rclpy

`rclpy` (ROS Client Library for Python) provides a Pythonic interface to ROS 2 functionality:

*   **Node creation and lifecycle management**
*   **Publisher and subscriber implementation**
*   **Service and action clients/servers**
*   **Parameter handling**
*   **Timer and callback management**
*   **Logging and diagnostics**

### Why Python for Robotics?

| Advantage | Description |
|-----------|-------------|
| **Rapid Prototyping** | Quick iteration on algorithms and behaviors |
| **Rich Ecosystem** | NumPy, OpenCV, PyTorch for AI/ML integration |
| **Readability** | Easier to maintain and understand code |
| **Interoperability** | Easy integration with web APIs and databases |

### When to Use C++ Instead

*   Real-time control loops (< 1ms latency requirements)
*   Low-level hardware drivers
*   Performance-critical perception pipelines
*   Embedded systems with limited resources

---

## Creating a ROS 2 Python Package

### Package Creation Command

```bash
# Navigate to your workspace source directory
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python humanoid_control \
    --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# Package structure created:
# humanoid_control/
# ├── humanoid_control/
# │   └── __init__.py
# ├── resource/
# │   └── humanoid_control
# ├── test/
# │   ├── test_copyright.py
# │   ├── test_flake8.py
# │   └── test_pep257.py
# ├── package.xml
# ├── setup.cfg
# └── setup.py
```

### Understanding Package Files

#### package.xml

Defines package metadata and dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>0.1.0</version>
  <description>Control package for humanoid robots</description>
  <maintainer email="developer@example.com">Developer Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### setup.py

Configures the Python package build:

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file for package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer Name',
    maintainer_email='developer@example.com',
    description='Control package for humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = humanoid_control.joint_controller:main',
            'balance_monitor = humanoid_control.balance_monitor:main',
            'gait_planner = humanoid_control.gait_planner:main',
        ],
    },
)
```

#### setup.cfg

Specifies installation directories:

```ini
[develop]
script_dir=$base/lib/humanoid_control

[install]
install_scripts=$base/lib/humanoid_control
```

---

## Building a Complete Humanoid Control Package

### Package Structure

```text
humanoid_control/
├── humanoid_control/
│   ├── __init__.py
│   ├── joint_controller.py
│   ├── balance_monitor.py
│   ├── gait_planner.py
│   └── utils/
│       ├── __init__.py
│       ├── kinematics.py
│       └── transformations.py
├── launch/
│   ├── humanoid_bringup.launch.py
│   └── simulation.launch.py
├── config/
│   ├── joint_limits.yaml
│   └── controller_params.yaml
├── resource/
│   └── humanoid_control
├── test/
│   ├── test_joint_controller.py
│   └── test_kinematics.py
├── package.xml
├── setup.cfg
└── setup.py
```

### Joint Controller Node

```python
#!/usr/bin/env python3
# humanoid_control/joint_controller.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration
import numpy as np

class JointController(Node):
    """
    Humanoid joint controller node.

    Subscribes to joint commands and publishes joint states.
    Provides services for enabling/disabling motors.
    """

    def __init__(self):
        super().__init__('joint_controller')

        # Declare parameters
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('joint_names', [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee_pitch', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee_pitch', 'right_ankle_pitch', 'right_ankle_roll'
        ])

        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.joint_names = self.get_parameter('joint_names').value
        self.num_joints = len(self.joint_names)

        # Initialize state
        self.current_positions = np.zeros(self.num_joints)
        self.target_positions = np.zeros(self.num_joints)
        self.motors_enabled = False

        # QoS for reliable command delivery
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            command_qos
        )

        # Services
        self.enable_srv = self.create_service(
            SetBool, '/enable_motors', self.enable_motors_callback)

        # Control loop timer
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'Joint controller started with {self.num_joints} joints '
            f'at {self.control_rate} Hz'
        )

    def trajectory_callback(self, msg: JointTrajectory):
        """Handle incoming trajectory commands."""
        if not self.motors_enabled:
            self.get_logger().warn('Motors disabled, ignoring trajectory')
            return

        if len(msg.points) == 0:
            return

        # Extract target from first point (simplified)
        point = msg.points[0]
        for i, name in enumerate(msg.joint_names):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.target_positions[idx] = point.positions[i]

        self.get_logger().debug(f'New trajectory received')

    def enable_motors_callback(self, request, response):
        """Handle motor enable/disable requests."""
        self.motors_enabled = request.data

        if self.motors_enabled:
            self.get_logger().info('Motors ENABLED')
            # Reset targets to current positions
            self.target_positions = self.current_positions.copy()
        else:
            self.get_logger().info('Motors DISABLED')

        response.success = True
        response.message = f'Motors {"enabled" if self.motors_enabled else "disabled"}'
        return response

    def control_loop(self):
        """Main control loop - runs at control_rate Hz."""
        if self.motors_enabled:
            # Simple P controller (replace with actual motor commands)
            error = self.target_positions - self.current_positions
            self.current_positions += 0.1 * error  # Gain = 0.1

        # Publish joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions.tolist()
        msg.velocity = [0.0] * self.num_joints
        msg.effort = [0.0] * self.num_joints

        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down joint controller')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Balance Monitor Node

```python
#!/usr/bin/env python3
# humanoid_control/balance_monitor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3Stamped
import numpy as np
from scipy.spatial.transform import Rotation

class BalanceMonitor(Node):
    """
    Monitors humanoid balance using IMU data.

    Publishes balance status and center of mass estimates.
    """

    def __init__(self):
        super().__init__('balance_monitor')

        # Parameters
        self.declare_parameter('roll_threshold', 0.3)   # radians
        self.declare_parameter('pitch_threshold', 0.3)  # radians
        self.declare_parameter('filter_alpha', 0.1)     # Complementary filter

        self.roll_threshold = self.get_parameter('roll_threshold').value
        self.pitch_threshold = self.get_parameter('pitch_threshold').value
        self.filter_alpha = self.get_parameter('filter_alpha').value

        # State
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.is_balanced = True

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.balance_pub = self.create_publisher(
            Bool, '/balance_status', 10)

        self.orientation_pub = self.create_publisher(
            Vector3Stamped, '/orientation_euler', 10)

        self.stability_pub = self.create_publisher(
            Float32, '/stability_margin', 10)

        # Timer for publishing status
        self.timer = self.create_timer(0.05, self.publish_status)  # 20 Hz

        self.get_logger().info('Balance monitor started')

    def imu_callback(self, msg: Imu):
        """Process IMU data and update orientation."""
        # Convert quaternion to euler angles
        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]

        rotation = Rotation.from_quat(q)
        euler = rotation.as_euler('xyz')

        # Low-pass filter
        alpha = self.filter_alpha
        self.roll = alpha * euler[0] + (1 - alpha) * self.roll
        self.pitch = alpha * euler[1] + (1 - alpha) * self.pitch
        self.yaw = alpha * euler[2] + (1 - alpha) * self.yaw

        # Check balance
        self.is_balanced = (
            abs(self.roll) < self.roll_threshold and
            abs(self.pitch) < self.pitch_threshold
        )

    def publish_status(self):
        """Publish balance status at regular intervals."""
        # Balance status
        balance_msg = Bool()
        balance_msg.data = self.is_balanced
        self.balance_pub.publish(balance_msg)

        # Euler angles
        orientation_msg = Vector3Stamped()
        orientation_msg.header.stamp = self.get_clock().now().to_msg()
        orientation_msg.header.frame_id = 'base_link'
        orientation_msg.vector.x = self.roll
        orientation_msg.vector.y = self.pitch
        orientation_msg.vector.z = self.yaw
        self.orientation_pub.publish(orientation_msg)

        # Stability margin (distance to threshold)
        roll_margin = self.roll_threshold - abs(self.roll)
        pitch_margin = self.pitch_threshold - abs(self.pitch)
        margin = min(roll_margin, pitch_margin)

        stability_msg = Float32()
        stability_msg.data = max(0.0, margin)
        self.stability_pub.publish(stability_msg)

        if not self.is_balanced:
            self.get_logger().warn(
                f'Balance warning: roll={np.degrees(self.roll):.1f}, '
                f'pitch={np.degrees(self.pitch):.1f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = BalanceMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Configuration Files

### Joint Limits (config/joint_limits.yaml)

```yaml
joint_limits:
  left_hip_yaw:
    min: -0.5
    max: 0.5
    velocity: 2.0
    effort: 50.0

  left_hip_roll:
    min: -0.3
    max: 0.3
    velocity: 2.0
    effort: 50.0

  left_hip_pitch:
    min: -1.5
    max: 0.5
    velocity: 3.0
    effort: 80.0

  left_knee_pitch:
    min: 0.0
    max: 2.5
    velocity: 3.0
    effort: 80.0

  # ... similar for other joints
```

### Controller Parameters (config/controller_params.yaml)

```yaml
joint_controller:
  ros__parameters:
    control_rate: 100.0
    joint_names:
      - left_hip_yaw
      - left_hip_roll
      - left_hip_pitch
      - left_knee_pitch
      - left_ankle_pitch
      - left_ankle_roll
      - right_hip_yaw
      - right_hip_roll
      - right_hip_pitch
      - right_knee_pitch
      - right_ankle_pitch
      - right_ankle_roll

balance_monitor:
  ros__parameters:
    roll_threshold: 0.3
    pitch_threshold: 0.3
    filter_alpha: 0.1
```

---

## Launch Files

### Humanoid Bringup Launch

```python
# launch/humanoid_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('humanoid_control')

    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'controller_params.yaml'
        ]),
        description='Path to controller configuration file'
    )

    # Nodes
    joint_controller_node = Node(
        package='humanoid_control',
        executable='joint_controller',
        name='joint_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    balance_monitor_node = Node(
        package='humanoid_control',
        executable='balance_monitor',
        name='balance_monitor',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        use_sim_arg,
        config_file_arg,
        joint_controller_node,
        balance_monitor_node,
    ])
```

---

## Building and Running

### Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select humanoid_control

# Source the workspace
source install/setup.bash
```

### Run Individual Nodes

```bash
# Run joint controller
ros2 run humanoid_control joint_controller

# Run balance monitor
ros2 run humanoid_control balance_monitor

# Run with parameters
ros2 run humanoid_control joint_controller \
    --ros-args -p control_rate:=50.0
```

### Launch the System

```bash
# Launch all nodes
ros2 launch humanoid_control humanoid_bringup.launch.py

# Launch with custom config
ros2 launch humanoid_control humanoid_bringup.launch.py \
    config_file:=/path/to/custom_params.yaml
```

---

## Testing Your Package

### Unit Tests

```python
# test/test_joint_controller.py

import pytest
import rclpy
from humanoid_control.joint_controller import JointController

@pytest.fixture
def node():
    rclpy.init()
    node = JointController()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_initial_state(node):
    """Test initial state of joint controller."""
    assert node.motors_enabled == False
    assert len(node.joint_names) == 12
    assert node.control_rate == 100.0

def test_motor_enable(node):
    """Test motor enable functionality."""
    from std_srvs.srv import SetBool

    request = SetBool.Request()
    request.data = True

    response = SetBool.Response()
    response = node.enable_motors_callback(request, response)

    assert response.success == True
    assert node.motors_enabled == True
```

### Running Tests

```bash
# Run all tests
colcon test --packages-select humanoid_control

# View test results
colcon test-result --verbose

# Run with pytest directly
cd ~/ros2_ws/src/humanoid_control
pytest test/ -v
```

---

## Best Practices for rclpy Packages

### 1. Node Design

*   Use descriptive parameter names with defaults
*   Implement proper shutdown handling
*   Use logging levels appropriately

### 2. Code Organization

*   Separate utility functions into modules
*   Keep node files focused on ROS communication
*   Use type hints for better code clarity

### 3. Configuration

*   Store all tunable values in YAML files
*   Use launch arguments for runtime configuration
*   Document parameter units and valid ranges

### 4. Error Handling

```python
def trajectory_callback(self, msg):
    try:
        # Process trajectory
        self.process_trajectory(msg)
    except ValueError as e:
        self.get_logger().error(f'Invalid trajectory: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error: {e}')
```

### 5. Resource Management

```python
def __init__(self):
    super().__init__('my_node')
    # Initialize resources

def destroy_node(self):
    # Clean up resources before destruction
    self.get_logger().info('Cleaning up...')
    super().destroy_node()
```

---

## Summary

This chapter covered the complete workflow for creating ROS 2 Python packages:

*   Package structure and configuration files
*   Building nodes with publishers, subscribers, and services
*   Managing parameters and configuration
*   Creating launch files for system deployment
*   Testing and best practices

With these skills, you can now build modular, reusable components for humanoid robot control systems. In the next chapter, we will explore URDF for defining humanoid robot models.
