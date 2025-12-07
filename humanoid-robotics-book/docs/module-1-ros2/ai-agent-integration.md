# Chapter 9: AI Agent Integration

This chapter explores how to integrate AI agents with ROS 2 to create intelligent humanoid behaviors. You will learn to connect large language models, decision-making systems, and behavior planners to your robot's control stack.

---

## The AI-Robot Architecture

Modern humanoid robots combine traditional control with AI reasoning:

```text
┌─────────────────────────────────────────────────────────────┐
│                    AI Agent Layer                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │    LLM      │  │   Vision    │  │   Speech    │         │
│  │  Planner    │  │   Model     │  │  (Whisper)  │         │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘         │
└─────────┼────────────────┼────────────────┼─────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 Middleware                         │
│  /ai/commands    /perception/objects    /speech/text        │
└─────────────────────────────────────────────────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────┐
│                    Robot Control Layer                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │   Motion    │  │    Nav2     │  │  Arm/Hand   │         │
│  │  Planner    │  │  Navigation │  │  Controller │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

---

## Building an AI Command Interface

### AI Command Message Definition

Create `msg/AICommand.msg`:

```text
# AICommand.msg - High-level AI commands for humanoid

std_msgs/Header header

# Command type
string command_type    # "navigate", "pick", "speak", "gesture"

# Target information
string target_name     # Object or location name
geometry_msgs/Pose target_pose

# Natural language instruction
string instruction

# Priority and constraints
int32 priority         # 0-10, higher = more urgent
float32 timeout        # Max execution time in seconds

# Metadata
string source          # "voice", "llm", "user_interface"
```

### AI Agent Node

```python
#!/usr/bin/env python3
# ai_agent_node.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import json

class AIAgentNode(Node):
    """
    Central AI agent that interprets commands and coordinates robot actions.
    """

    def __init__(self):
        super().__init__('ai_agent')

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/ai/natural_language_command',
            self.command_callback, 10)

        self.perception_sub = self.create_subscription(
            String, '/perception/scene_description',
            self.perception_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/ai/agent_status', 10)

        self.speech_pub = self.create_publisher(
            String, '/speech/say', 10)

        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # State
        self.scene_context = {}
        self.current_task = None
        self.known_locations = {
            'kitchen': {'x': 2.0, 'y': 1.0},
            'living_room': {'x': 0.0, 'y': 3.0},
            'bedroom': {'x': -2.0, 'y': 2.0},
            'entrance': {'x': 0.0, 'y': 0.0}
        }

        self.get_logger().info('AI Agent initialized')

    def command_callback(self, msg: String):
        """Process natural language commands."""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        # Parse and execute command
        if 'go to' in command or 'navigate to' in command:
            self.handle_navigation_command(command)
        elif 'pick up' in command or 'grab' in command:
            self.handle_manipulation_command(command)
        elif 'say' in command or 'speak' in command:
            self.handle_speech_command(command)
        else:
            self.publish_status('unknown_command', command)

    def handle_navigation_command(self, command: str):
        """Parse and execute navigation commands."""
        # Extract location from command
        location = None
        for loc_name in self.known_locations:
            if loc_name in command:
                location = loc_name
                break

        if location:
            coords = self.known_locations[location]
            self.navigate_to(coords['x'], coords['y'], location)
        else:
            self.speak(f"I don't know where that location is.")

    def navigate_to(self, x: float, y: float, location_name: str):
        """Send navigation goal."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Navigation server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.speak(f"Navigating to {location_name}")
        self.current_task = f'navigate_to_{location_name}'

        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        remaining = feedback_msg.feedback.distance_remaining
        self.publish_status('navigating', f'Distance: {remaining:.2f}m')

    def nav_goal_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle navigation completion."""
        self.speak("I have arrived at the destination")
        self.current_task = None

    def handle_manipulation_command(self, command: str):
        """Parse and execute manipulation commands."""
        self.speak("Manipulation commands are not yet implemented")

    def handle_speech_command(self, command: str):
        """Extract and speak text."""
        # Extract text after "say" or "speak"
        for trigger in ['say ', 'speak ']:
            if trigger in command:
                text = command.split(trigger, 1)[1]
                self.speak(text)
                return

    def speak(self, text: str):
        """Publish text for speech synthesis."""
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        self.get_logger().info(f'Speaking: {text}')

    def perception_callback(self, msg: String):
        """Update scene context from perception."""
        try:
            self.scene_context = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def publish_status(self, status: str, details: str = ''):
        """Publish agent status."""
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'details': details,
            'current_task': self.current_task
        })
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AIAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Integrating Large Language Models

### LLM Planner Node

```python
#!/usr/bin/env python3
# llm_planner_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json

class LLMPlannerNode(Node):
    """
    Uses LLM to generate task plans from natural language instructions.
    """

    def __init__(self):
        super().__init__('llm_planner')

        # Parameters
        self.declare_parameter('model_name', 'gpt-4')
        self.declare_parameter('api_endpoint', 'http://localhost:8000/v1')

        self.model_name = self.get_parameter('model_name').value

        # Subscribers
        self.instruction_sub = self.create_subscription(
            String, '/ai/instruction',
            self.instruction_callback, 10)

        # Publishers
        self.plan_pub = self.create_publisher(
            String, '/ai/task_plan', 10)

        # System prompt for robotics planning
        self.system_prompt = """You are a task planner for a humanoid robot.
Given a natural language instruction, output a JSON plan with steps.

Available robot actions:
- navigate(location): Move to a named location
- pick(object): Pick up an object
- place(location): Place held object at location
- speak(text): Say something
- wait(seconds): Wait for specified time

Example output:
{
  "task": "fetch_cup",
  "steps": [
    {"action": "navigate", "params": {"location": "kitchen"}},
    {"action": "pick", "params": {"object": "cup"}},
    {"action": "navigate", "params": {"location": "living_room"}},
    {"action": "place", "params": {"location": "table"}}
  ]
}

Output only valid JSON, no explanations."""

        self.get_logger().info('LLM Planner ready')

    def instruction_callback(self, msg: String):
        """Process instruction and generate plan."""
        instruction = msg.data
        self.get_logger().info(f'Planning for: {instruction}')

        # Generate plan (simulated - replace with actual LLM call)
        plan = self.generate_plan(instruction)

        if plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

    def generate_plan(self, instruction: str) -> dict:
        """
        Generate a task plan using LLM.
        In production, this would call an actual LLM API.
        """
        # Simulated planning logic
        instruction_lower = instruction.lower()

        if 'bring' in instruction_lower or 'fetch' in instruction_lower:
            # Extract object and destination
            return {
                "task": "fetch_object",
                "steps": [
                    {"action": "speak", "params": {"text": "I'll fetch that for you"}},
                    {"action": "navigate", "params": {"location": "kitchen"}},
                    {"action": "pick", "params": {"object": "item"}},
                    {"action": "navigate", "params": {"location": "user"}},
                    {"action": "speak", "params": {"text": "Here you go"}}
                ]
            }

        elif 'clean' in instruction_lower:
            return {
                "task": "clean_area",
                "steps": [
                    {"action": "speak", "params": {"text": "Starting cleaning"}},
                    {"action": "navigate", "params": {"location": "target_area"}},
                    {"action": "wait", "params": {"seconds": 5}},
                    {"action": "speak", "params": {"text": "Cleaning complete"}}
                ]
            }

        else:
            return {
                "task": "unknown",
                "steps": [
                    {"action": "speak", "params": {"text": "I'm not sure how to do that"}}
                ]
            }

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Task Execution Framework

### Task Executor Node

```python
#!/usr/bin/env python3
# task_executor_node.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import json
from enum import Enum

class TaskState(Enum):
    IDLE = 'idle'
    EXECUTING = 'executing'
    PAUSED = 'paused'
    COMPLETED = 'completed'
    FAILED = 'failed'

class TaskExecutorNode(Node):
    """
    Executes task plans step by step.
    """

    def __init__(self):
        super().__init__('task_executor')

        # Subscribers
        self.plan_sub = self.create_subscription(
            String, '/ai/task_plan',
            self.plan_callback, 10)

        self.control_sub = self.create_subscription(
            String, '/ai/executor_control',
            self.control_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/ai/executor_status', 10)

        self.speech_pub = self.create_publisher(
            String, '/speech/say', 10)

        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # State
        self.state = TaskState.IDLE
        self.current_plan = None
        self.current_step = 0

        # Location database
        self.locations = {
            'kitchen': (2.0, 1.0),
            'living_room': (0.0, 3.0),
            'user': (1.0, 0.0)
        }

        # Timer for step execution
        self.exec_timer = None

        self.get_logger().info('Task Executor ready')

    def plan_callback(self, msg: String):
        """Receive and start executing a new plan."""
        try:
            self.current_plan = json.loads(msg.data)
            self.current_step = 0
            self.state = TaskState.EXECUTING
            self.get_logger().info(
                f'Starting plan: {self.current_plan.get("task", "unknown")}')
            self.execute_next_step()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid plan JSON: {e}')

    def control_callback(self, msg: String):
        """Handle executor control commands."""
        command = msg.data.lower()

        if command == 'pause':
            self.state = TaskState.PAUSED
        elif command == 'resume':
            if self.state == TaskState.PAUSED:
                self.state = TaskState.EXECUTING
                self.execute_next_step()
        elif command == 'cancel':
            self.state = TaskState.IDLE
            self.current_plan = None

        self.publish_status()

    def execute_next_step(self):
        """Execute the next step in the plan."""
        if self.state != TaskState.EXECUTING:
            return

        if not self.current_plan or 'steps' not in self.current_plan:
            self.state = TaskState.COMPLETED
            return

        steps = self.current_plan['steps']

        if self.current_step >= len(steps):
            self.state = TaskState.COMPLETED
            self.get_logger().info('Plan completed')
            self.publish_status()
            return

        step = steps[self.current_step]
        action = step.get('action', '')
        params = step.get('params', {})

        self.get_logger().info(
            f'Executing step {self.current_step + 1}/{len(steps)}: {action}')

        # Execute action
        if action == 'navigate':
            self.execute_navigate(params)
        elif action == 'speak':
            self.execute_speak(params)
            self.step_completed()
        elif action == 'wait':
            self.execute_wait(params)
        elif action == 'pick':
            self.execute_pick(params)
        elif action == 'place':
            self.execute_place(params)
        else:
            self.get_logger().warn(f'Unknown action: {action}')
            self.step_completed()

        self.publish_status()

    def execute_navigate(self, params: dict):
        """Execute navigation action."""
        location = params.get('location', '')
        if location in self.locations:
            x, y = self.locations[location]
            self.send_navigation_goal(x, y)
        else:
            self.get_logger().warn(f'Unknown location: {location}')
            self.step_completed()

    def send_navigation_goal(self, x: float, y: float):
        """Send navigation goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 not available')
            self.step_completed()
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_callback)

    def nav_goal_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.step_completed())
        else:
            self.step_completed()

    def execute_speak(self, params: dict):
        """Execute speech action."""
        text = params.get('text', '')
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)

    def execute_wait(self, params: dict):
        """Execute wait action."""
        seconds = params.get('seconds', 1.0)
        self.exec_timer = self.create_timer(
            seconds, self.wait_completed)

    def wait_completed(self):
        """Called when wait is complete."""
        if self.exec_timer:
            self.exec_timer.cancel()
            self.exec_timer = None
        self.step_completed()

    def execute_pick(self, params: dict):
        """Execute pick action (placeholder)."""
        obj = params.get('object', 'unknown')
        self.get_logger().info(f'Picking up: {obj}')
        # In real implementation, call manipulation action
        self.step_completed()

    def execute_place(self, params: dict):
        """Execute place action (placeholder)."""
        loc = params.get('location', 'unknown')
        self.get_logger().info(f'Placing at: {loc}')
        self.step_completed()

    def step_completed(self):
        """Mark current step as completed and move to next."""
        self.current_step += 1
        if self.state == TaskState.EXECUTING:
            self.execute_next_step()

    def publish_status(self):
        """Publish executor status."""
        status = {
            'state': self.state.value,
            'task': self.current_plan.get('task') if self.current_plan else None,
            'step': self.current_step,
            'total_steps': len(self.current_plan.get('steps', [])) if self.current_plan else 0
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Behavior Trees for Complex Tasks

Behavior trees provide a modular way to compose robot behaviors:

```python
#!/usr/bin/env python3
# behavior_tree_node.py

from enum import Enum
from typing import Callable, List

class NodeStatus(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    RUNNING = 'running'

class BTNode:
    """Base class for behavior tree nodes."""
    def tick(self) -> NodeStatus:
        raise NotImplementedError

class SequenceNode(BTNode):
    """Executes children in sequence until one fails."""
    def __init__(self, children: List[BTNode]):
        self.children = children
        self.current = 0

    def tick(self) -> NodeStatus:
        while self.current < len(self.children):
            status = self.children[self.current].tick()
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self.current = 0
                return NodeStatus.FAILURE
            self.current += 1
        self.current = 0
        return NodeStatus.SUCCESS

class SelectorNode(BTNode):
    """Tries children until one succeeds."""
    def __init__(self, children: List[BTNode]):
        self.children = children

    def tick(self) -> NodeStatus:
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.FAILURE:
                return status
        return NodeStatus.FAILURE

class ActionNode(BTNode):
    """Executes an action function."""
    def __init__(self, action: Callable[[], NodeStatus]):
        self.action = action

    def tick(self) -> NodeStatus:
        return self.action()

class ConditionNode(BTNode):
    """Checks a condition."""
    def __init__(self, condition: Callable[[], bool]):
        self.condition = condition

    def tick(self) -> NodeStatus:
        return NodeStatus.SUCCESS if self.condition() else NodeStatus.FAILURE

# Example: Fetch object behavior tree
def create_fetch_behavior(robot):
    """Create a behavior tree for fetching objects."""
    return SequenceNode([
        # First, check if we know where the object is
        SelectorNode([
            ConditionNode(lambda: robot.object_location_known()),
            ActionNode(lambda: robot.search_for_object())
        ]),
        # Navigate to object
        ActionNode(lambda: robot.navigate_to_object()),
        # Pick up object
        ActionNode(lambda: robot.pick_object()),
        # Return to user
        ActionNode(lambda: robot.navigate_to_user()),
        # Hand over object
        ActionNode(lambda: robot.hand_over())
    ])
```

---

## Voice Command Integration

### Voice Command Node

```python
#!/usr/bin/env python3
# voice_command_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class VoiceCommandNode(Node):
    """
    Processes voice commands using speech recognition.
    """

    def __init__(self):
        super().__init__('voice_command')

        # Parameters
        self.declare_parameter('wake_word', 'robot')
        self.declare_parameter('language', 'en-US')

        self.wake_word = self.get_parameter('wake_word').value

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData, '/audio/input',
            self.audio_callback, 10)

        self.transcription_sub = self.create_subscription(
            String, '/speech/transcription',
            self.transcription_callback, 10)

        # Publishers
        self.command_pub = self.create_publisher(
            String, '/ai/natural_language_command', 10)

        self.status_pub = self.create_publisher(
            String, '/voice/status', 10)

        # State
        self.listening = False
        self.buffer = []

        self.get_logger().info(
            f'Voice command ready. Wake word: "{self.wake_word}"')

    def audio_callback(self, msg: AudioData):
        """Process incoming audio (placeholder for VAD)."""
        pass

    def transcription_callback(self, msg: String):
        """Process transcribed text."""
        text = msg.data.lower().strip()

        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        # Check for wake word
        if self.wake_word in text:
            self.listening = True
            # Extract command after wake word
            parts = text.split(self.wake_word, 1)
            if len(parts) > 1 and parts[1].strip():
                self.process_command(parts[1].strip())
            else:
                self.publish_status('listening')
        elif self.listening:
            self.process_command(text)
            self.listening = False

    def process_command(self, command: str):
        """Process and publish voice command."""
        # Filter out noise/partial words
        if len(command) < 3:
            return

        self.get_logger().info(f'Command: "{command}"')

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        self.publish_status('command_received')

    def publish_status(self, status: str):
        """Publish voice recognition status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## System Launch File

```python
# launch/ai_agent_bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_ai',
            executable='ai_agent_node',
            name='ai_agent',
            output='screen'
        ),
        Node(
            package='humanoid_ai',
            executable='llm_planner_node',
            name='llm_planner',
            parameters=[{
                'model_name': 'gpt-4',
            }],
            output='screen'
        ),
        Node(
            package='humanoid_ai',
            executable='task_executor_node',
            name='task_executor',
            output='screen'
        ),
        Node(
            package='humanoid_ai',
            executable='voice_command_node',
            name='voice_command',
            parameters=[{
                'wake_word': 'robot',
            }],
            output='screen'
        ),
    ])
```

---

## Summary

This chapter covered integrating AI agents with ROS 2:

*   **AI-Robot Architecture**: Layered design connecting AI reasoning to robot control
*   **Command Processing**: Natural language to robot actions
*   **LLM Planning**: Using language models for task decomposition
*   **Task Execution**: Step-by-step plan execution with monitoring
*   **Behavior Trees**: Modular behavior composition
*   **Voice Commands**: Speech-to-command pipeline

These patterns form the foundation for building intelligent humanoid robots that can understand and execute complex instructions. In the next module, we will explore digital twin simulation for testing these AI behaviors safely.
