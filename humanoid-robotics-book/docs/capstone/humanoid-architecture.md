# Chapter 23: Autonomous Humanoid Architecture

This chapter presents a complete software architecture for building an autonomous humanoid assistant, integrating all concepts from previous modules.

---

## System Architecture Overview

```text
┌─────────────────────────────────────────────────────────────────────┐
│                   Autonomous Humanoid System                        │
├─────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    High-Level AI Layer                       │   │
│  │  ┌─────────┐  ┌─────────────┐  ┌───────────┐  ┌──────────┐  │   │
│  │  │   LLM   │  │   Vision    │  │   Speech  │  │  Memory  │  │   │
│  │  │ Planner │  │   Language  │  │ Interface │  │  System  │  │   │
│  │  └─────────┘  └─────────────┘  └───────────┘  └──────────┘  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    Behavior Layer                            │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │   │
│  │  │  Behavior   │  │    State    │  │      Safety         │  │   │
│  │  │    Trees    │  │   Machine   │  │     Monitor         │  │   │
│  │  └─────────────┘  └─────────────┘  └─────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   Navigation Layer                           │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │   │
│  │  │   SLAM   │  │   Nav2   │  │ Obstacle │  │   Motion    │  │   │
│  │  │          │  │ Planner  │  │ Avoidance│  │  Planning   │  │   │
│  │  └──────────┘  └──────────┘  └──────────┘  └─────────────┘  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   Control Layer                              │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │   │
│  │  │ Balance  │  │  Joint   │  │   Arm    │  │    Gait     │  │   │
│  │  │ Control  │  │ Control  │  │  Control │  │  Generator  │  │   │
│  │  └──────────┘  └──────────┘  └──────────┘  └─────────────┘  │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   Hardware Abstraction                       │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │   │
│  │  │  Motors  │  │ Sensors  │  │ Cameras  │  │     IMU     │  │   │
│  │  └──────────┘  └──────────┘  └──────────┘  └─────────────┘  │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Package Structure

```text
humanoid_ws/
├── src/
│   ├── humanoid_description/     # URDF and meshes
│   ├── humanoid_hardware/        # Hardware drivers
│   ├── humanoid_control/         # Joint and balance control
│   ├── humanoid_navigation/      # SLAM and Nav2
│   ├── humanoid_perception/      # Vision and detection
│   ├── humanoid_voice/           # Speech I/O
│   ├── humanoid_ai/              # LLM and planning
│   ├── humanoid_behaviors/       # Behavior trees
│   └── humanoid_bringup/         # Launch files
├── config/
│   ├── robot_params.yaml
│   ├── nav2_params.yaml
│   └── ai_params.yaml
└── launch/
    ├── robot.launch.py
    ├── simulation.launch.py
    └── full_system.launch.py
```

---

## Core Components

### State Manager

```python
#!/usr/bin/env python3
# state_manager.py

from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any

class RobotState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PLANNING = "planning"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    SPEAKING = "speaking"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"

@dataclass
class WorldState:
    robot_location: str = "unknown"
    robot_pose: Dict[str, float] = None
    held_object: Optional[str] = None
    battery_level: float = 100.0
    detected_objects: Dict[str, Any] = None
    detected_humans: list = None
    current_task: Optional[str] = None

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.robot_state = RobotState.IDLE
        self.world_state = WorldState()
        self.state_history = []

        # Publishers
        self.state_pub = self.create_publisher(
            String, '/robot/state', 10)

        # Subscribers for state updates
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery', self.battery_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/detection/objects', self.detection_callback, 10)

        # State change callbacks
        self.state_callbacks = {}

    def set_state(self, new_state: RobotState, reason: str = ""):
        old_state = self.robot_state
        self.robot_state = new_state

        self.state_history.append({
            'from': old_state,
            'to': new_state,
            'reason': reason,
            'time': time.time()
        })

        self.publish_state()

        # Notify callbacks
        if new_state in self.state_callbacks:
            for callback in self.state_callbacks[new_state]:
                callback(old_state, new_state)

    def register_state_callback(self, state: RobotState, callback):
        if state not in self.state_callbacks:
            self.state_callbacks[state] = []
        self.state_callbacks[state].append(callback)

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.robot_state.value,
            'world': {
                'location': self.world_state.robot_location,
                'held_object': self.world_state.held_object,
                'battery': self.world_state.battery_level,
            }
        })
        self.state_pub.publish(msg)
```

### Safety Monitor

```python
#!/usr/bin/env python3
# safety_monitor.py

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        self.emergency_stop = False
        self.safety_violations = []

        # Safety limits
        self.max_velocity = 0.5  # m/s
        self.max_tilt = 0.4      # radians
        self.min_battery = 10.0  # percent
        self.obstacle_threshold = 0.3  # meters

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery', self.battery_callback, 10)

        # Publishers
        self.safe_cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel_safe', 10)
        self.estop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10)

    def imu_callback(self, msg):
        # Check tilt
        roll, pitch = self.quaternion_to_euler(msg.orientation)[:2]
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.trigger_safety('excessive_tilt', f'Roll: {roll}, Pitch: {pitch}')

    def cmd_vel_callback(self, msg):
        # Limit velocities
        safe_msg = Twist()
        safe_msg.linear.x = max(-self.max_velocity,
                               min(self.max_velocity, msg.linear.x))
        safe_msg.angular.z = max(-1.0, min(1.0, msg.angular.z))

        if not self.emergency_stop:
            self.safe_cmd_vel_pub.publish(safe_msg)

    def scan_callback(self, msg):
        # Check for close obstacles
        min_range = min(msg.ranges)
        if min_range < self.obstacle_threshold:
            self.trigger_safety('obstacle_too_close', f'Distance: {min_range}')

    def battery_callback(self, msg):
        if msg.percentage < self.min_battery:
            self.trigger_safety('low_battery', f'Battery: {msg.percentage}%')

    def trigger_safety(self, violation_type, details):
        self.safety_violations.append({
            'type': violation_type,
            'details': details,
            'time': time.time()
        })

        if violation_type in ['excessive_tilt', 'obstacle_too_close']:
            self.emergency_stop = True
            estop_msg = Bool()
            estop_msg.data = True
            self.estop_pub.publish(estop_msg)
            self.get_logger().error(f'EMERGENCY STOP: {violation_type}')
```

---

## Integration Layer

### Main Orchestrator

```python
#!/usr/bin/env python3
# humanoid_orchestrator.py

class HumanoidOrchestrator(Node):
    """Main orchestration node coordinating all subsystems."""

    def __init__(self):
        super().__init__('humanoid_orchestrator')

        # Initialize components
        self.state_manager = StateManager()
        self.safety_monitor = SafetyMonitor()

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/robot/command', self.command_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)

        # Service clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = ActionClient(self, MoveArm, 'move_arm')

        # Main loop timer
        self.main_timer = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.get_logger().info('Humanoid orchestrator initialized')

    def main_loop(self):
        """Main control loop."""
        state = self.state_manager.robot_state

        if state == RobotState.EMERGENCY_STOP:
            self.handle_emergency()
        elif state == RobotState.IDLE:
            self.handle_idle()
        elif state == RobotState.NAVIGATING:
            self.handle_navigation()
        elif state == RobotState.MANIPULATING:
            self.handle_manipulation()

    def command_callback(self, msg):
        """Process high-level commands."""
        command = json.loads(msg.data)
        action = command.get('action')

        if action == 'navigate':
            self.start_navigation(command['target'])
        elif action == 'pick':
            self.start_manipulation('pick', command['object'])
        elif action == 'speak':
            self.speak(command['text'])

    def start_navigation(self, target):
        self.state_manager.set_state(RobotState.NAVIGATING, f'Going to {target}')
        # Send navigation goal

    def estop_callback(self, msg):
        if msg.data:
            self.state_manager.set_state(RobotState.EMERGENCY_STOP, 'E-Stop triggered')
```

---

## Configuration

### Robot Parameters (config/robot_params.yaml)

```yaml
robot:
  name: "humanoid_assistant"
  version: "1.0.0"

hardware:
  joint_count: 22
  control_frequency: 500  # Hz

  joint_limits:
    left_hip_pitch: [-1.5, 0.5]
    left_knee_pitch: [0.0, 2.5]
    # ... other joints

safety:
  max_velocity: 0.5
  max_acceleration: 1.0
  max_tilt: 0.4
  min_battery: 10.0
  obstacle_clearance: 0.3

behavior:
  idle_timeout: 30.0
  task_timeout: 300.0
  retry_count: 3
```

---

## Launch Configuration

```python
# launch/full_system.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('humanoid_bringup')

    return LaunchDescription([
        # Hardware
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_dir, '/launch/hardware.launch.py'])
        ),

        # Perception
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_dir, '/launch/perception.launch.py'])
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_dir, '/launch/navigation.launch.py'])
        ),

        # AI and Planning
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_dir, '/launch/ai.launch.py'])
        ),

        # Main orchestrator
        Node(
            package='humanoid_bringup',
            executable='humanoid_orchestrator',
            name='orchestrator',
            output='screen'
        ),

        # Safety monitor
        Node(
            package='humanoid_bringup',
            executable='safety_monitor',
            name='safety',
            output='screen'
        ),
    ])
```

---

## Summary

This chapter presented the autonomous humanoid architecture:

*   **Layered design**: AI, behavior, navigation, control, hardware
*   **State management**: Centralized state tracking
*   **Safety systems**: Monitoring and emergency stop
*   **Integration**: Orchestration and coordination

In the next chapter, we will implement voice command functionality.
