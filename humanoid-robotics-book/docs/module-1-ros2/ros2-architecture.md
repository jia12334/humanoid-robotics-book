# Chapter 5: ROS 2 Architecture

ROS 2 (Robot Operating System 2) serves as the foundational middleware for modern robotics development. This chapter explores the architecture, design principles, and core concepts that make ROS 2 the preferred framework for building scalable, real-time robotic systems, including humanoid robots.

---

## What is ROS 2?

ROS 2 is not an operating system in the traditional sense. Instead, it is a **middleware framework** that provides:

*   **Communication Infrastructure**: Standardized message passing between software components
*   **Hardware Abstraction**: Unified interfaces for sensors, actuators, and controllers
*   **Software Ecosystem**: Thousands of packages for perception, navigation, manipulation, and more
*   **Development Tools**: Visualization, debugging, simulation, and deployment utilities

### Evolution from ROS 1 to ROS 2

ROS 1 revolutionized robotics research but had limitations for production deployments:

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| **Communication** | Custom TCP/UDP (TCPROS) | DDS (Data Distribution Service) |
| **Real-time Support** | Limited | Native real-time capable |
| **Security** | Minimal | Built-in DDS Security |
| **Multi-robot** | Complex setup | Native support |
| **OS Support** | Linux only | Linux, Windows, macOS |
| **Lifecycle Management** | None | Managed node lifecycles |

---

## Core Architectural Concepts

### The ROS 2 Graph

The ROS 2 computation graph is a network of processes (nodes) that communicate through well-defined interfaces:

```text
┌─────────────────────────────────────────────────────────────┐
│                      ROS 2 Graph                            │
│                                                             │
│  ┌──────────┐    Topic     ┌──────────┐                    │
│  │  Camera  │─────────────▶│ Detector │                    │
│  │   Node   │  /image_raw  │   Node   │                    │
│  └──────────┘              └────┬─────┘                    │
│                                 │                          │
│                          /detections                       │
│                                 ▼                          │
│  ┌──────────┐  Service   ┌──────────┐    Topic    ┌──────┐│
│  │  Planner │◀──────────▶│ Decision │────────────▶│ Motor││
│  │   Node   │ /plan_path │   Node   │  /cmd_vel   │ Node ││
│  └──────────┘            └──────────┘             └──────┘│
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

1.  **Nodes**: Modular processes that perform specific computations
2.  **Topics**: Named buses for publish-subscribe messaging
3.  **Services**: Request-response communication patterns
4.  **Actions**: Long-running tasks with feedback and cancellation
5.  **Parameters**: Runtime-configurable node settings

---

## Data Distribution Service (DDS)

ROS 2 is built on DDS, an industry-standard middleware for real-time systems.

### Why DDS?

*   **Decentralized Discovery**: No single point of failure (no rosmaster)
*   **Quality of Service (QoS)**: Fine-grained control over communication reliability
*   **Real-time Performance**: Deterministic message delivery
*   **Security**: Authentication, encryption, and access control

### DDS Implementations

ROS 2 supports multiple DDS vendors through the RMW (ROS Middleware) abstraction:

| Implementation | Vendor | License | Use Case |
|----------------|--------|---------|----------|
| **Fast DDS** | eProsima | Apache 2.0 | Default, general purpose |
| **Cyclone DDS** | Eclipse | EPL 2.0 | High performance |
| **Connext DDS** | RTI | Commercial | Enterprise, real-time |
| **GurumDDS** | GurumNetworks | Commercial | Embedded systems |

```bash
# Set DDS implementation (example)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Quality of Service (QoS)

QoS policies determine how messages are handled, enabling trade-offs between reliability and performance.

### Key QoS Policies

| Policy | Options | Description |
|--------|---------|-------------|
| **Reliability** | RELIABLE / BEST_EFFORT | Guaranteed vs. fire-and-forget delivery |
| **Durability** | TRANSIENT_LOCAL / VOLATILE | Late-joining subscriber behavior |
| **History** | KEEP_LAST(n) / KEEP_ALL | Message queue behavior |
| **Deadline** | Duration | Expected message frequency |
| **Liveliness** | AUTOMATIC / MANUAL | Node health checking |

### QoS Profiles for Robotics

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Sensor data - high frequency, tolerate drops
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Commands - must be delivered reliably
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

---

## Node Architecture

Nodes are the fundamental units of computation in ROS 2. Each node should have a single, well-defined purpose.

### Node Lifecycle (Managed Nodes)

ROS 2 introduces lifecycle nodes with defined states:

```text
                    ┌──────────────┐
                    │   Created    │
                    └──────┬───────┘
                           │ configure()
                           ▼
                    ┌──────────────┐
          ┌────────│  Inactive    │◀───────┐
          │        └──────┬───────┘        │
          │               │ activate()     │ deactivate()
          │               ▼                │
          │        ┌──────────────┐        │
          │        │    Active    │────────┘
          │        └──────┬───────┘
          │               │ cleanup()
          │               ▼
          │        ┌──────────────┐
          └───────▶│  Finalized   │
                   └──────────────┘
```

### Node Composition

Multiple nodes can run in a single process for efficiency:

```python
# Component-based node loading
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(camera_node)
executor.add_node(detector_node)
executor.add_node(planner_node)
executor.spin()
```

---

## Communication Patterns

### Topics (Publish-Subscribe)

Best for continuous data streams like sensor readings:

```python
# Publisher
self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
self.publisher.publish(image_msg)

# Subscriber
self.subscription = self.create_subscription(
    Image, '/camera/image_raw', self.image_callback, 10)
```

### Services (Request-Response)

Best for synchronous operations:

```python
# Service server
self.srv = self.create_service(
    SetBool, '/enable_motor', self.enable_callback)

# Service client
self.client = self.create_client(SetBool, '/enable_motor')
request = SetBool.Request()
request.data = True
future = self.client.call_async(request)
```

### Actions (Long-running Tasks)

Best for tasks with feedback and cancellation:

```python
# Action server
self._action_server = ActionServer(
    self, NavigateToPose, 'navigate_to_pose',
    execute_callback=self.execute_callback,
    goal_callback=self.goal_callback,
    cancel_callback=self.cancel_callback)

# Action client
self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
goal_handle = await self._action_client.send_goal_async(goal_msg)
```

---

## Package Structure

ROS 2 organizes code into packages with a standardized structure:

```text
my_robot_package/
├── CMakeLists.txt          # Build configuration (C++)
├── package.xml             # Package metadata and dependencies
├── setup.py                # Build configuration (Python)
├── setup.cfg               # Python package config
├── resource/               # Package marker files
│   └── my_robot_package
├── my_robot_package/       # Python source code
│   ├── __init__.py
│   ├── camera_node.py
│   └── detector_node.py
├── src/                    # C++ source code
│   └── motor_controller.cpp
├── include/                # C++ headers
│   └── my_robot_package/
├── launch/                 # Launch files
│   └── robot.launch.py
├── config/                 # Configuration files
│   └── params.yaml
├── msg/                    # Custom message definitions
├── srv/                    # Custom service definitions
└── action/                 # Custom action definitions
```

---

## Launch System

The ROS 2 launch system orchestrates multiple nodes and configurations:

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),

        Node(
            package='my_robot_package',
            executable='camera_node',
            name='camera',
            parameters=[{'resolution': '1080p'}],
            remappings=[('/camera/image', '/robot/camera/image')]
        ),

        Node(
            package='my_robot_package',
            executable='detector_node',
            name='detector',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),
    ])
```

---

## ROS 2 for Humanoid Robots

Humanoid robots present unique architectural challenges that ROS 2 addresses:

### Multi-DOF Control

Humanoids have many degrees of freedom requiring coordinated control:

```text
Humanoid Control Architecture
├── /joint_states (sensor_msgs/JointState)
│   └── All joint positions, velocities, efforts
├── /joint_commands (trajectory_msgs/JointTrajectory)
│   └── Commanded joint trajectories
├── /imu/data (sensor_msgs/Imu)
│   └── Balance and orientation
└── /foot_contacts (custom_msgs/FootContact)
    └── Ground contact detection
```

### Real-time Requirements

For bipedal balance, real-time control loops are essential:

```bash
# Enable real-time scheduling
sudo setcap cap_sys_nice+ep /path/to/ros2_node

# Configure DDS for real-time
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

### Distributed Computing

Complex humanoids distribute computation across multiple processors:

```text
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Head Unit     │     │   Torso Unit    │     │   Leg Units     │
│  (Jetson Orin)  │     │   (Main PC)     │     │  (Embedded)     │
├─────────────────┤     ├─────────────────┤     ├─────────────────┤
│ - Vision        │     │ - Planning      │     │ - Motor Control │
│ - Speech        │◀───▶│ - Coordination  │◀───▶│ - Balance       │
│ - Face Tracking │     │ - State Machine │     │ - Foot Sensing  │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        │                       │                       │
        └───────────────────────┴───────────────────────┘
                         ROS 2 DDS Network
```

---

## Tools and Utilities

### Command Line Tools

```bash
# List active nodes
ros2 node list

# View node information
ros2 node info /camera_node

# List topics
ros2 topic list

# Echo topic data
ros2 topic echo /camera/image_raw

# Call a service
ros2 service call /enable_motor std_srvs/srv/SetBool "{data: true}"

# View parameter
ros2 param get /camera_node resolution
```

### Visualization with RViz2

RViz2 provides 3D visualization of robot state, sensor data, and planning:

```bash
# Launch RViz2
ros2 run rviz2 rviz2

# With saved configuration
ros2 run rviz2 rviz2 -d /path/to/config.rviz
```

### Introspection with rqt

```bash
# Node graph visualization
ros2 run rqt_graph rqt_graph

# Topic monitoring
ros2 run rqt_topic rqt_topic

# Parameter reconfiguration
ros2 run rqt_reconfigure rqt_reconfigure
```

---

## Best Practices for ROS 2 Development

### 1. Node Design

*   **Single Responsibility**: Each node should do one thing well
*   **Configurable**: Use parameters for runtime configuration
*   **Testable**: Design for unit and integration testing

### 2. Communication

*   **Choose Appropriate Patterns**: Topics for streams, services for queries, actions for tasks
*   **Define Clear Interfaces**: Document message formats and expected behavior
*   **Use Standard Messages**: Prefer existing message types when possible

### 3. Error Handling

*   **Graceful Degradation**: Handle node failures without system crash
*   **Logging**: Use appropriate log levels (DEBUG, INFO, WARN, ERROR)
*   **Health Monitoring**: Implement diagnostics for critical nodes

### 4. Performance

*   **Profile First**: Identify bottlenecks before optimizing
*   **Use Composition**: Reduce inter-process communication overhead
*   **Optimize QoS**: Match QoS settings to data characteristics

---

## Summary

ROS 2 architecture provides the foundation for building sophisticated humanoid robotics systems. Its DDS-based communication, lifecycle management, and real-time capabilities make it suitable for both research and production deployments. Understanding these core concepts is essential before diving into practical node development in the following chapters.

In the next chapter, we will explore nodes, topics, and services in depth with hands-on examples that you can run on your development machine.
