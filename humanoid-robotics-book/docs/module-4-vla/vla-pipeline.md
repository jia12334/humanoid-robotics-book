# Chapter 22: VLA Pipeline Examples

This chapter presents complete Vision-Language-Action pipeline implementations for humanoid robots, demonstrating end-to-end systems from perception to action.

---

## VLA Pipeline Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                    Vision-Language-Action Pipeline              │
│                                                                 │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐  │
│  │  Vision  │───▶│ Language │───▶│ Planning │───▶│  Action  │  │
│  │  Input   │    │  Input   │    │  Module  │    │  Output  │  │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘  │
│       │              │               │               │         │
│       ▼              ▼               ▼               ▼         │
│   Object         Command         Task Plan      Motor Cmds     │
│   Detection      Parsing         Generation     Trajectories   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Example 1: Object Fetch Pipeline

```python
#!/usr/bin/env python3
# fetch_pipeline.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import json

class FetchPipeline(Node):
    """Complete pipeline for 'fetch me the [object]' commands."""

    def __init__(self):
        super().__init__('fetch_pipeline')

        # State
        self.detected_objects = {}
        self.current_task = None

        # Vision
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/detection/objects', self.detection_callback, 10)

        # Language
        self.command_sub = self.create_subscription(
            String, '/voice/command', self.command_callback, 10)

        # Action
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arm_pub = self.create_publisher(String, '/arm/command', 10)
        self.speech_pub = self.create_publisher(String, '/speech/say', 10)

        self.get_logger().info('Fetch pipeline ready')

    def detection_callback(self, msg):
        """Update detected objects."""
        self.detected_objects = json.loads(msg.data)

    def command_callback(self, msg):
        """Process voice command."""
        command = msg.data.lower()

        if 'fetch' in command or 'bring' in command or 'get' in command:
            # Extract object name
            target = self.extract_target(command)
            if target:
                self.execute_fetch(target)

    def extract_target(self, command):
        """Extract target object from command."""
        words = command.split()
        # Simple extraction - look for object after action word
        for i, word in enumerate(words):
            if word in ['fetch', 'bring', 'get', 'grab']:
                if i + 1 < len(words):
                    return ' '.join(words[i+1:]).strip('the ')
        return None

    def execute_fetch(self, target):
        """Execute fetch sequence."""
        self.speak(f"Looking for {target}")

        # Check if object is visible
        if target in self.detected_objects:
            obj = self.detected_objects[target]
            self.speak(f"Found {target}, fetching it now")
            self.fetch_visible_object(obj)
        else:
            # Search for object
            self.speak(f"Searching for {target}")
            self.search_for_object(target)

    def fetch_visible_object(self, obj):
        """Fetch an object currently in view."""
        # Navigate to object
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = 'map'
        nav_goal.pose.position.x = obj['position']['x'] - 0.5
        nav_goal.pose.position.y = obj['position']['y']
        nav_goal.pose.orientation.w = 1.0
        self.nav_pub.publish(nav_goal)

        # After navigation, pick up
        arm_cmd = String()
        arm_cmd.data = json.dumps({
            'action': 'pick',
            'target_pose': obj['position']
        })
        self.arm_pub.publish(arm_cmd)

    def search_for_object(self, target):
        """Search for object in known locations."""
        search_locations = ['kitchen', 'living_room', 'bedroom']
        # Implement search behavior
        pass

    def speak(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
```

---

## Example 2: Instruction Following

```python
#!/usr/bin/env python3
# instruction_follower.py

class InstructionFollower(Node):
    """Follow complex multi-step instructions."""

    def __init__(self):
        super().__init__('instruction_follower')

        self.llm_client = OpenAI()
        self.current_plan = None
        self.step_index = 0

        self.command_sub = self.create_subscription(
            String, '/voice/command', self.command_callback, 10)

        self.action_pub = self.create_publisher(String, '/robot/action', 10)

    def command_callback(self, msg):
        instruction = msg.data
        plan = self.generate_plan(instruction)

        if plan:
            self.execute_plan(plan)

    def generate_plan(self, instruction):
        """Use LLM to generate action plan."""
        prompt = f"""Convert this instruction into robot actions:
"{instruction}"

Available actions:
- navigate(location)
- pick(object)
- place(location)
- open(object)
- speak(text)

Output JSON array of steps."""

        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1
        )

        try:
            return json.loads(response.choices[0].message.content)
        except:
            return None

    def execute_plan(self, plan):
        """Execute plan step by step."""
        for step in plan:
            self.get_logger().info(f'Executing: {step}')

            action_msg = String()
            action_msg.data = json.dumps(step)
            self.action_pub.publish(action_msg)

            # Wait for completion
            success = self.wait_for_action_completion()
            if not success:
                self.speak("I encountered a problem, stopping")
                break

    def wait_for_action_completion(self):
        # Implementation
        return True
```

---

## Example 3: Visual Question Answering

```python
#!/usr/bin/env python3
# visual_qa.py

class VisualQA(Node):
    """Answer questions about the visual scene."""

    def __init__(self):
        super().__init__('visual_qa')

        # Vision-Language Model
        self.vlm = self.load_vlm_model()
        self.current_image = None

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.question_sub = self.create_subscription(
            String, '/voice/question', self.question_callback, 10)

        self.answer_pub = self.create_publisher(String, '/speech/say', 10)

    def load_vlm_model(self):
        # Load a vision-language model like LLaVA or BLIP
        from transformers import BlipProcessor, BlipForQuestionAnswering
        processor = BlipProcessor.from_pretrained("Salesforce/blip-vqa-base")
        model = BlipForQuestionAnswering.from_pretrained("Salesforce/blip-vqa-base")
        return {'processor': processor, 'model': model}

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def question_callback(self, msg):
        question = msg.data

        if self.current_image is None:
            self.speak("I cannot see anything right now")
            return

        answer = self.answer_question(question)
        self.speak(answer)

    def answer_question(self, question):
        """Use VLM to answer question about scene."""
        processor = self.vlm['processor']
        model = self.vlm['model']

        pil_image = PILImage.fromarray(self.current_image)
        inputs = processor(pil_image, question, return_tensors="pt")

        outputs = model.generate(**inputs)
        answer = processor.decode(outputs[0], skip_special_tokens=True)

        return answer

    def speak(self, text):
        msg = String()
        msg.data = text
        self.answer_pub.publish(msg)
```

---

## Example 4: Complete VLA System

```python
#!/usr/bin/env python3
# complete_vla_system.py

class CompleteVLASystem(Node):
    """Integrated Vision-Language-Action system."""

    def __init__(self):
        super().__init__('complete_vla')

        # Components
        self.perception = PerceptionModule(self)
        self.language = LanguageModule(self)
        self.planner = PlanningModule(self)
        self.executor = ExecutionModule(self)

        # State
        self.world_state = WorldState()
        self.conversation_history = []

        # Main processing timer
        self.timer = self.create_timer(0.1, self.process_loop)

    def process_loop(self):
        """Main processing loop."""
        # Update perception
        self.perception.update(self.world_state)

        # Process any pending commands
        if self.language.has_pending_input():
            input_data = self.language.get_input()
            self.handle_input(input_data)

    def handle_input(self, input_data):
        """Handle multi-modal input."""
        # Determine intent
        intent = self.language.understand_intent(
            input_data,
            self.world_state,
            self.conversation_history
        )

        if intent['type'] == 'question':
            response = self.answer_question(intent)
            self.language.speak(response)

        elif intent['type'] == 'command':
            plan = self.planner.create_plan(intent, self.world_state)
            self.executor.execute(plan)

        elif intent['type'] == 'clarification_needed':
            self.language.ask_clarification(intent['question'])

        # Update conversation history
        self.conversation_history.append({
            'input': input_data,
            'intent': intent,
            'timestamp': time.time()
        })


class PerceptionModule:
    def __init__(self, node):
        self.node = node
        self.object_detector = ObjectDetector()
        self.scene_understander = SceneUnderstander()

    def update(self, world_state):
        # Update detected objects
        world_state.objects = self.object_detector.detect()
        world_state.scene_description = self.scene_understander.describe()


class LanguageModule:
    def __init__(self, node):
        self.node = node
        self.asr = WhisperASR()
        self.tts = TextToSpeech()
        self.intent_classifier = IntentClassifier()

    def understand_intent(self, input_data, world_state, history):
        # Use LLM for intent understanding
        return self.intent_classifier.classify(
            input_data, world_state, history
        )


class PlanningModule:
    def __init__(self, node):
        self.node = node
        self.llm_planner = LLMPlanner()

    def create_plan(self, intent, world_state):
        return self.llm_planner.plan(intent, world_state)


class ExecutionModule:
    def __init__(self, node):
        self.node = node

    def execute(self, plan):
        for step in plan['steps']:
            self.execute_step(step)
```

---

## Launch Configuration

```python
# launch/vla_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Perception
        Node(package='humanoid_vla', executable='object_detector'),
        Node(package='humanoid_vla', executable='scene_understanding'),

        # Language
        Node(package='humanoid_vla', executable='whisper_node'),
        Node(package='humanoid_vla', executable='tts_node'),
        Node(package='humanoid_vla', executable='llm_planner'),

        # Fusion
        Node(package='humanoid_vla', executable='multimodal_fusion'),

        # Main VLA
        Node(package='humanoid_vla', executable='complete_vla_system'),
    ])
```

---

## Summary

This chapter presented complete VLA pipeline examples:

*   **Object fetch**: Voice command to robot action
*   **Instruction following**: LLM-based plan generation
*   **Visual QA**: Answering questions about scenes
*   **Complete system**: Integrated VLA architecture

These examples demonstrate practical VLA implementation for humanoid robots. In the next module, we will apply everything in a capstone project.
