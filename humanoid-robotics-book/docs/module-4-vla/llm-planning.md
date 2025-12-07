# Chapter 20: Natural Language Planning with LLMs

Large Language Models enable high-level task planning from natural language instructions. This chapter covers integrating LLMs for robot task decomposition and planning.

---

## LLM Planning Architecture

```text
┌──────────────────┐     ┌─────────────────────────────────────┐
│  User Instruction │────▶│            LLM Planner              │
│  "Make me coffee" │     │  ┌─────────┐  ┌─────────────────┐  │
└──────────────────┘     │  │ Context │  │  Task Library   │  │
                         │  └─────────┘  └─────────────────┘  │
                         └────────────────┬────────────────────┘
                                          │
                                          ▼
                         ┌─────────────────────────────────────┐
                         │          Structured Plan            │
                         │  1. Navigate to kitchen             │
                         │  2. Locate coffee machine           │
                         │  3. Pick up cup                     │
                         │  4. Place cup under dispenser       │
                         │  5. Press brew button               │
                         │  6. Wait for completion             │
                         │  7. Deliver coffee to user          │
                         └─────────────────────────────────────┘
```

---

## LLM Integration Options

| Option | Latency | Cost | Privacy |
|--------|---------|------|---------|
| **OpenAI API** | Low | Per-token | Cloud |
| **Claude API** | Low | Per-token | Cloud |
| **Local LLaMA** | Medium | Hardware | Full |
| **Ollama** | Medium | Hardware | Full |

---

## Basic LLM Planner

```python
#!/usr/bin/env python3
# llm_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from openai import OpenAI

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('api_key', '')

        self.model = self.get_parameter('model').value
        api_key = self.get_parameter('api_key').value

        self.client = OpenAI(api_key=api_key)

        # System prompt defining robot capabilities
        self.system_prompt = """You are a task planner for a humanoid robot.

Available robot actions:
- navigate(location): Move to a location
- pick(object): Pick up an object
- place(surface): Place held object
- open(object): Open a door/drawer/container
- close(object): Close a door/drawer/container
- press(button): Press a button
- wait(seconds): Wait for specified time
- speak(text): Say something to the user

Known locations: kitchen, living_room, bedroom, bathroom, entrance

Output a JSON plan with this structure:
{
  "task": "brief_task_name",
  "steps": [
    {"action": "action_name", "params": {"param": "value"}},
    ...
  ],
  "preconditions": ["list of required conditions"],
  "estimated_time": seconds
}

Only output valid JSON, no explanations."""

        # Subscribers
        self.instruction_sub = self.create_subscription(
            String, '/voice/command', self.instruction_callback, 10)

        # Publishers
        self.plan_pub = self.create_publisher(String, '/robot/plan', 10)
        self.status_pub = self.create_publisher(String, '/planner/status', 10)

    def instruction_callback(self, msg):
        instruction = msg.data
        self.get_logger().info(f'Planning for: "{instruction}"')
        self.publish_status('planning')

        plan = self.generate_plan(instruction)

        if plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            self.publish_status('plan_ready')
        else:
            self.publish_status('planning_failed')

    def generate_plan(self, instruction):
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": instruction}
                ],
                temperature=0.1,
                max_tokens=1000
            )

            content = response.choices[0].message.content
            plan = json.loads(content)
            return plan

        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
            return None

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
```

---

## Local LLM with Ollama

```python
#!/usr/bin/env python3
# local_llm_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests

class LocalLLMPlanner(Node):
    def __init__(self):
        super().__init__('local_llm_planner')

        self.declare_parameter('model', 'llama3.1')
        self.declare_parameter('ollama_url', 'http://localhost:11434')

        self.model = self.get_parameter('model').value
        self.ollama_url = self.get_parameter('ollama_url').value

        self.system_prompt = """You are a robot task planner.
Output JSON plans for robot actions.
Actions: navigate, pick, place, speak, wait
Output only valid JSON."""

        self.sub = self.create_subscription(
            String, '/voice/command', self.plan_callback, 10)
        self.pub = self.create_publisher(String, '/robot/plan', 10)

    def plan_callback(self, msg):
        plan = self.query_ollama(msg.data)
        if plan:
            out = String()
            out.data = json.dumps(plan)
            self.pub.publish(out)

    def query_ollama(self, instruction):
        try:
            response = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model': self.model,
                    'prompt': f'{self.system_prompt}\n\nTask: {instruction}',
                    'stream': False,
                    'format': 'json'
                }
            )
            result = response.json()
            return json.loads(result['response'])
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return None
```

---

## Context-Aware Planning

```python
class ContextAwarePlanner(Node):
    def __init__(self):
        super().__init__('context_planner')

        self.world_state = {
            'robot_location': 'living_room',
            'held_object': None,
            'battery_level': 80,
            'known_objects': {},
            'recent_actions': []
        }

        # Subscribe to state updates
        self.location_sub = self.create_subscription(
            String, '/robot/location', self.location_callback, 10)
        self.perception_sub = self.create_subscription(
            String, '/perception/objects', self.perception_callback, 10)

    def build_context_prompt(self, instruction):
        context = f"""Current robot state:
- Location: {self.world_state['robot_location']}
- Holding: {self.world_state['held_object'] or 'nothing'}
- Battery: {self.world_state['battery_level']}%
- Visible objects: {list(self.world_state['known_objects'].keys())}
- Recent actions: {self.world_state['recent_actions'][-5:]}

User instruction: {instruction}

Generate a plan considering the current state."""

        return context

    def location_callback(self, msg):
        self.world_state['robot_location'] = msg.data

    def perception_callback(self, msg):
        objects = json.loads(msg.data)
        self.world_state['known_objects'] = objects
```

---

## Plan Validation

```python
class PlanValidator:
    def __init__(self, robot_capabilities):
        self.capabilities = robot_capabilities
        self.known_locations = ['kitchen', 'living_room', 'bedroom']

    def validate_plan(self, plan):
        errors = []

        for i, step in enumerate(plan.get('steps', [])):
            action = step.get('action')
            params = step.get('params', {})

            # Check action exists
            if action not in self.capabilities:
                errors.append(f'Step {i}: Unknown action "{action}"')
                continue

            # Validate parameters
            if action == 'navigate':
                if params.get('location') not in self.known_locations:
                    errors.append(f'Step {i}: Unknown location')

            elif action == 'wait':
                if params.get('seconds', 0) > 300:
                    errors.append(f'Step {i}: Wait too long')

        return len(errors) == 0, errors

    def fix_plan(self, plan, errors):
        """Attempt to fix common plan issues."""
        fixed_plan = plan.copy()
        # Implementation for auto-fixing
        return fixed_plan
```

---

## Plan Execution with Feedback

```python
class PlanExecutor(Node):
    def __init__(self):
        super().__init__('plan_executor')

        self.current_plan = None
        self.current_step = 0
        self.execution_state = 'idle'

        self.plan_sub = self.create_subscription(
            String, '/robot/plan', self.plan_callback, 10)

        self.feedback_pub = self.create_publisher(
            String, '/execution/feedback', 10)

    async def execute_plan(self, plan):
        self.current_plan = plan
        self.current_step = 0

        for i, step in enumerate(plan['steps']):
            self.current_step = i
            self.publish_feedback(f'Executing step {i+1}: {step["action"]}')

            success = await self.execute_step(step)

            if not success:
                self.publish_feedback(f'Step {i+1} failed, replanning...')
                return False

        self.publish_feedback('Plan completed successfully')
        return True

    async def execute_step(self, step):
        action = step['action']
        params = step.get('params', {})

        action_map = {
            'navigate': self.execute_navigate,
            'pick': self.execute_pick,
            'place': self.execute_place,
            'speak': self.execute_speak,
            'wait': self.execute_wait,
        }

        if action in action_map:
            return await action_map[action](params)
        return False

    def publish_feedback(self, message):
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)
```

---

## Summary

This chapter covered LLM-based planning:

*   **Architecture**: LLM integration patterns
*   **Cloud vs Local**: OpenAI API and Ollama
*   **Context awareness**: Including robot state in prompts
*   **Validation**: Checking plan feasibility
*   **Execution**: Running plans with feedback

LLM planners enable complex task understanding. In the next chapter, we will explore multi-modal interaction combining vision and language.
