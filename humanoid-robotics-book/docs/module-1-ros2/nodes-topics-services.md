# Chapter 6: Nodes, Topics, and Services

This chapter provides hands-on exploration of ROS 2's core communication primitives: nodes, topics, and services. Through practical examples, you will learn how to build the fundamental building blocks of any robotics system, including humanoid robots.

---

## Understanding Nodes

A **node** is a single-purpose process that performs computation. In a humanoid robot, nodes might include:

*   Camera image processor
*   Joint state publisher
*   Balance controller
*   Voice command interpreter
*   Path planner

### Creating Your First Node

Let's create a simple node that logs messages:

```python
#!/usr/bin/env python3
# simple_node.py

import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node has started!')

        # Create a timer that fires every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer fired: count = {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Node

```bash
# Make the script executable
chmod +x simple_node.py

# Run directly
python3 simple_node.py

# Or through ROS 2 (if installed as a package)
ros2 run my_package simple_node
```

### Node Naming and Namespaces

Nodes can be organized using namespaces to avoid conflicts:

```bash
# Run with namespace
ros2 run my_package camera_node --ros-args -r __ns:=/robot1

# Run with remapped name
ros2 run my_package camera_node --ros-args -r __node:=front_camera
```

---

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, many-to-many communication. Publishers send messages to a topic, and any number of subscribers can receive them.

### Topic Architecture

```text
┌───────────┐                              ┌───────────┐
│ Publisher │──┐                      ┌───▶│Subscriber1│
│   Node    │  │    /sensor/data      │    │   Node    │
└───────────┘  │   ┌────────────┐     │    └───────────┘
               └──▶│   Topic    │─────┤
┌───────────┐  ┌──▶│            │     │    ┌───────────┐
│ Publisher │──┘   └────────────┘     └───▶│Subscriber2│
│   Node    │                              │   Node    │
└───────────┘                              └───────────┘
```

### Creating a Publisher

```python
#!/usr/bin/env python3
# joint_state_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher
        self.publisher = self.create_publisher(
            JointState,           # Message type
            '/joint_states',      # Topic name
            10                    # Queue size
        )

        # Timer for periodic publishing
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50 Hz
        self.time = 0.0

        # Define humanoid joints
        self.joint_names = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee_pitch', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
            'right_knee_pitch', 'right_ankle_pitch', 'right_ankle_roll'
        ]

        self.get_logger().info('Joint state publisher started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Simulate walking motion
        self.time += 0.02
        positions = []
        for i, name in enumerate(self.joint_names):
            # Simple sinusoidal motion
            phase = 0.0 if 'left' in name else math.pi
            position = 0.2 * math.sin(2.0 * self.time + phase)
            positions.append(position)

        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber

```python
#!/usr/bin/env python3
# joint_state_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,                    # Message type
            '/joint_states',               # Topic name
            self.joint_states_callback,    # Callback function
            10                             # Queue size
        )

        self.get_logger().info('Joint state subscriber started')

    def joint_states_callback(self, msg):
        # Process received joint states
        self.get_logger().info(f'Received {len(msg.name)} joint states')

        for name, position in zip(msg.name, msg.position):
            self.get_logger().debug(f'  {name}: {position:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Message Types for Humanoids

| Message Type | Package | Use Case |
|--------------|---------|----------|
| `JointState` | sensor_msgs | Joint positions, velocities, efforts |
| `Imu` | sensor_msgs | IMU data for balance |
| `Image` | sensor_msgs | Camera images |
| `PointCloud2` | sensor_msgs | 3D point cloud data |
| `Twist` | geometry_msgs | Velocity commands |
| `Pose` | geometry_msgs | Position and orientation |
| `Wrench` | geometry_msgs | Force/torque sensor data |
| `JointTrajectory` | trajectory_msgs | Motion trajectories |

---

## Services: Request-Response Communication

Services provide synchronous, one-to-one communication for operations that need immediate feedback.

### Service Architecture

```text
┌────────────┐    Request     ┌────────────┐
│   Client   │───────────────▶│   Server   │
│    Node    │                │    Node    │
│            │◀───────────────│            │
└────────────┘    Response    └────────────┘
```

### Creating a Service Server

```python
#!/usr/bin/env python3
# motor_enable_server.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MotorEnableServer(Node):
    def __init__(self):
        super().__init__('motor_enable_server')

        # Create service server
        self.srv = self.create_service(
            SetBool,                      # Service type
            '/enable_motors',             # Service name
            self.enable_motors_callback   # Callback function
        )

        self.motors_enabled = False
        self.get_logger().info('Motor enable service ready')

    def enable_motors_callback(self, request, response):
        self.motors_enabled = request.data

        if self.motors_enabled:
            self.get_logger().info('Motors ENABLED')
            response.success = True
            response.message = 'Motors enabled successfully'
        else:
            self.get_logger().info('Motors DISABLED')
            response.success = True
            response.message = 'Motors disabled successfully'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorEnableServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python
#!/usr/bin/env python3
# motor_enable_client.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MotorEnableClient(Node):
    def __init__(self):
        super().__init__('motor_enable_client')

        # Create service client
        self.client = self.create_client(SetBool, '/enable_motors')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for motor enable service...')

        self.get_logger().info('Motor enable client ready')

    def send_request(self, enable):
        request = SetBool.Request()
        request.data = enable

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = MotorEnableClient()

    # Enable motors
    response = node.send_request(True)
    print(f'Response: success={response.success}, message="{response.message}"')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Service Types

| Service Type | Package | Use Case |
|--------------|---------|----------|
| `SetBool` | std_srvs | Toggle boolean states |
| `Trigger` | std_srvs | Trigger an action |
| `Empty` | std_srvs | No request/response data |
| `GetPlan` | nav_msgs | Request navigation path |
| `SetJointPositions` | custom | Set joint positions |

---

## Custom Messages and Services

For humanoid-specific needs, you may need custom message types.

### Defining a Custom Message

Create `msg/FootContact.msg`:

```text
# FootContact.msg - Foot contact sensor data

std_msgs/Header header

# Left foot contact points
bool left_heel_contact
bool left_toe_contact
float32 left_force      # Total force in Newtons

# Right foot contact points
bool right_heel_contact
bool right_toe_contact
float32 right_force
```

### Defining a Custom Service

Create `srv/SetGaitMode.srv`:

```text
# SetGaitMode.srv - Change humanoid walking mode

# Request
string mode      # "walk", "run", "stand", "crouch"
float32 speed    # Target speed in m/s

---

# Response
bool success
string message
string current_mode
```

### Building Custom Messages

Update `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContact.msg"
  "srv/SetGaitMode.srv"
  DEPENDENCIES std_msgs
)
```

Update `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## Actions: Long-Running Tasks

Actions extend services for tasks that take time, provide feedback, and can be canceled.

### Action Architecture

```text
┌─────────────┐                        ┌─────────────┐
│   Action    │  ──Goal Request───▶    │   Action    │
│   Client    │  ◀───Goal Response──   │   Server    │
│             │                        │             │
│             │  ◀────Feedback─────    │             │
│             │                        │             │
│             │  ◀──Result Response──  │             │
└─────────────┘                        └─────────────┘
```

### Using the NavigateToPose Action

```python
#!/usr/bin/env python3
# navigate_client.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigateClient(Node):
    def __init__(self):
        super().__init__('navigate_client')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Simplified

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation completed!')

def main(args=None):
    rclpy.init(args=args)
    client = NavigateClient()
    client.send_goal(2.0, 1.0, 0.0)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

---

## Practical Exercise: Humanoid Status Monitor

Let's build a complete example that combines publishers, subscribers, and services:

```python
#!/usr/bin/env python3
# humanoid_status_monitor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3
import math

class HumanoidStatusMonitor(Node):
    def __init__(self):
        super().__init__('humanoid_status_monitor')

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publisher for status
        self.status_pub = self.create_publisher(
            Vector3, '/robot_status/balance', 10)

        # Service for status report
        self.status_srv = self.create_service(
            Trigger, '/get_status_report', self.status_report_callback)

        # State variables
        self.joint_positions = {}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.is_balanced = True

        # Timer for status publishing
        self.timer = self.create_timer(0.1, self.publish_balance_status)

        self.get_logger().info('Humanoid status monitor started')

    def joint_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def imu_callback(self, msg):
        # Convert quaternion to euler (simplified)
        q = msg.orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.orientation['roll'] = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.orientation['pitch'] = math.asin(max(-1, min(1, sinp)))

        # Check balance
        self.is_balanced = (
            abs(self.orientation['roll']) < 0.3 and
            abs(self.orientation['pitch']) < 0.3
        )

    def publish_balance_status(self):
        msg = Vector3()
        msg.x = self.orientation['roll']
        msg.y = self.orientation['pitch']
        msg.z = 1.0 if self.is_balanced else 0.0
        self.status_pub.publish(msg)

    def status_report_callback(self, request, response):
        report = []
        report.append(f'Balance: {"OK" if self.is_balanced else "UNSTABLE"}')
        report.append(f'Roll: {math.degrees(self.orientation["roll"]):.1f} deg')
        report.append(f'Pitch: {math.degrees(self.orientation["pitch"]):.1f} deg')
        report.append(f'Joints monitored: {len(self.joint_positions)}')

        response.success = True
        response.message = ' | '.join(report)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatusMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Command Line Tools Reference

### Topic Commands

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /joint_states

# Echo messages
ros2 topic echo /joint_states

# Publish a message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# Check publishing rate
ros2 topic hz /joint_states

# Show message type
ros2 topic type /joint_states
```

### Service Commands

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /enable_motors

# Call a service
ros2 service call /enable_motors std_srvs/srv/SetBool "{data: true}"

# Show service info
ros2 service info /enable_motors
```

### Node Commands

```bash
# List all nodes
ros2 node list

# Show node info
ros2 node info /humanoid_status_monitor
```

---

## Summary

This chapter covered the essential communication primitives in ROS 2:

*   **Nodes**: Single-purpose processes that form the building blocks of robotic systems
*   **Topics**: Publish-subscribe communication for streaming data like sensor readings
*   **Services**: Request-response communication for synchronous operations
*   **Actions**: Extended services for long-running tasks with feedback and cancellation

Understanding these patterns is fundamental to building complex humanoid robotics applications. In the next chapter, we will explore how to package these components into reusable Python packages using `rclpy`.
