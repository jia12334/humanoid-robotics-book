# Chapter 25: Path Planning and Navigation

This chapter implements autonomous navigation for the capstone humanoid assistant using Nav2.

---

## Navigation System

```python
#!/usr/bin/env python3
# capstone_navigation.py

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CapstoneNavigation(Node):
    def __init__(self):
        super().__init__('capstone_navigation')

        self.navigator = BasicNavigator()

        # Known locations
        self.locations = {
            'kitchen': {'x': 3.0, 'y': 1.0, 'yaw': 0.0},
            'living_room': {'x': 0.0, 'y': 2.0, 'yaw': 1.57},
            'bedroom': {'x': -2.0, 'y': 1.5, 'yaw': 3.14},
            'entrance': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        }

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/robot/parsed_command', self.command_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.speech_pub = self.create_publisher(String, '/speech/say', 10)

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Navigation ready')

    def command_callback(self, msg):
        command = json.loads(msg.data)

        if command['action'] == 'navigate':
            self.navigate_to(command['target'])

    def navigate_to(self, target):
        if target not in self.locations:
            self.speak(f"I don't know where {target} is")
            return

        location = self.locations[target]
        self.speak(f"Going to {target}")
        self.publish_status('navigating', target)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = location['x']
        goal.pose.position.y = location['y']
        goal.pose.orientation.w = 1.0

        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
                self.publish_status('navigating', f'{remaining:.1f}m remaining')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.speak(f"Arrived at {target}")
            self.publish_status('arrived', target)
        else:
            self.speak("Navigation failed")
            self.publish_status('failed', target)

    def publish_status(self, status, details=""):
        msg = String()
        msg.data = json.dumps({'status': status, 'details': details})
        self.status_pub.publish(msg)

    def speak(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CapstoneNavigation()
    rclpy.spin(node)
```

---

## Location Learning

```python
class LocationLearner(Node):
    def __init__(self):
        super().__init__('location_learner')

        self.locations = {}
        self.load_locations()

        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/voice/command', self.command_callback, 10)

        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg

    def command_callback(self, msg):
        text = msg.data.lower()

        if 'remember this' in text or 'save location' in text:
            # Extract name
            name = text.split('as')[-1].strip() if 'as' in text else 'unnamed'
            self.save_current_location(name)

    def save_current_location(self, name):
        if self.current_pose:
            self.locations[name] = {
                'x': self.current_pose.pose.position.x,
                'y': self.current_pose.pose.position.y,
            }
            self.save_locations()
            self.get_logger().info(f'Saved location: {name}')

    def save_locations(self):
        with open('locations.json', 'w') as f:
            json.dump(self.locations, f)

    def load_locations(self):
        try:
            with open('locations.json', 'r') as f:
                self.locations = json.load(f)
        except FileNotFoundError:
            self.locations = {}
```

---

## Summary

This chapter implemented navigation:

*   Nav2 integration
*   Location-based navigation
*   Dynamic location learning
*   Status feedback

In the next chapter, we will implement object detection and interaction.
