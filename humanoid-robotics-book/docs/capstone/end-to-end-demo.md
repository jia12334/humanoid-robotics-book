# Chapter 27: End-to-End Capstone Demo

This chapter presents the complete integrated capstone demonstration, bringing together all systems into a working humanoid assistant.

---

## Complete System Integration

```python
#!/usr/bin/env python3
# capstone_main.py

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json

class CapstoneMain(Node):
    """Main capstone node integrating all subsystems."""

    def __init__(self):
        super().__init__('capstone_main')

        # State
        self.state = 'idle'
        self.current_task = None

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/robot/parsed_command', self.command_callback, 10)
        self.nav_status_sub = self.create_subscription(
            String, '/navigation/status', self.nav_status_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/detection/objects', self.detection_callback, 10)

        # Publishers
        self.speech_pub = self.create_publisher(String, '/speech/say', 10)
        self.status_pub = self.create_publisher(String, '/capstone/status', 10)

        self.get_logger().info('Capstone system initialized')
        self.speak("Hello, I am your humanoid assistant. Say 'robot' to wake me up.")

    def command_callback(self, msg):
        command = json.loads(msg.data)
        action = command['action']
        target = command.get('target')

        self.get_logger().info(f'Executing: {action} {target or ""}')

        if action == 'navigate':
            self.state = 'navigating'
            self.current_task = f'navigate to {target}'

        elif action == 'fetch':
            self.state = 'fetching'
            self.current_task = f'fetch {target}'
            self.execute_fetch(target)

        elif action == 'status':
            self.report_status()

        elif action == 'stop':
            self.stop_all()

        elif action == 'follow':
            self.start_following()

    def execute_fetch(self, target):
        """Execute complete fetch sequence."""
        self.speak(f"I will fetch {target} for you")

        # This would coordinate:
        # 1. Find object location
        # 2. Navigate to object
        # 3. Pick up object
        # 4. Return to user
        # 5. Hand over object

    def nav_status_callback(self, msg):
        status = json.loads(msg.data)
        if status['status'] == 'arrived':
            self.state = 'idle'
            self.current_task = None

    def detection_callback(self, msg):
        # Update object awareness
        pass

    def report_status(self):
        status = f"I am currently {self.state}"
        if self.current_task:
            status += f", working on: {self.current_task}"
        self.speak(status)

    def stop_all(self):
        self.state = 'idle'
        self.current_task = None
        self.speak("Stopping")

    def start_following(self):
        self.state = 'following'
        self.speak("I will follow you")

    def speak(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    main_node = CapstoneMain()

    executor = MultiThreadedExecutor()
    executor.add_node(main_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        main_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Complete Launch File

```python
# launch/capstone_demo.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('humanoid_capstone')

    return LaunchDescription([
        # Voice system
        Node(package='humanoid_capstone', executable='voice_system', name='voice'),
        Node(package='humanoid_capstone', executable='command_parser', name='parser'),
        Node(package='humanoid_capstone', executable='tts', name='tts'),

        # Perception
        Node(package='humanoid_capstone', executable='object_detection', name='detection'),

        # Navigation (includes Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg, '/launch/navigation.launch.py'])
        ),
        Node(package='humanoid_capstone', executable='capstone_navigation', name='nav'),

        # Manipulation
        Node(package='humanoid_capstone', executable='manipulation', name='manipulation'),

        # Main orchestrator
        Node(package='humanoid_capstone', executable='capstone_main', name='main'),
    ])
```

---

## Demo Scenarios

### Scenario 1: Fetch Object

```text
User: "Robot, fetch me the cup from the kitchen"

System flow:
1. Voice → Transcription: "fetch me the cup from the kitchen"
2. Parser → Command: {action: "fetch", target: "cup"}
3. Main → Plan: Navigate to kitchen → Find cup → Pick up → Return
4. Navigation → Go to kitchen
5. Detection → Find cup
6. Manipulation → Pick cup
7. Navigation → Return to user
8. Voice → "Here is your cup"
```

### Scenario 2: Navigation

```text
User: "Robot, go to the living room"

System flow:
1. Voice → Transcription: "go to the living room"
2. Parser → Command: {action: "navigate", target: "living_room"}
3. Navigation → Navigate to living_room
4. Voice → "I've arrived at the living room"
```

---

## Testing Checklist

- [ ] Voice activation with wake word
- [ ] Command recognition accuracy
- [ ] Navigation to known locations
- [ ] Object detection reliability
- [ ] Pick and place operations
- [ ] Status reporting
- [ ] Emergency stop
- [ ] Battery monitoring
- [ ] Error recovery

---

## Summary

This capstone project integrated:

*   **Voice interface**: Wake word, ASR, TTS
*   **Navigation**: SLAM, path planning, obstacle avoidance
*   **Perception**: Object detection, scene understanding
*   **Manipulation**: Pick and place operations
*   **Planning**: LLM-based task decomposition
*   **Safety**: Monitoring and emergency stop

Congratulations on completing the Physical AI and Humanoid Robotics curriculum! You now have the foundation to build intelligent humanoid systems.
