# Chapter 19: Voice-to-Action using Whisper

This chapter covers implementing voice command interfaces for humanoid robots using OpenAI's Whisper model for speech recognition, enabling natural language control.

---

## Voice-to-Action Architecture

```text
┌────────────┐     ┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ Microphone │────▶│   Whisper   │────▶│   Command    │────▶│   Robot     │
│   Array    │     │   (ASR)     │     │   Parser     │     │   Actions   │
└────────────┘     └─────────────┘     └──────────────┘     └─────────────┘
                          │                   │
                          ▼                   ▼
                   Raw Transcription    Structured Command
                   "go to the kitchen"  {action: navigate,
                                         target: kitchen}
```

---

## Whisper Setup

### Installation

```bash
# Install Whisper
pip install openai-whisper

# Install with CUDA support
pip install openai-whisper torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install audio dependencies
sudo apt install ffmpeg portaudio19-dev
pip install sounddevice numpy
```

### Model Selection

| Model | Parameters | Speed | Accuracy | VRAM |
|-------|------------|-------|----------|------|
| tiny | 39M | Fastest | Good | ~1 GB |
| base | 74M | Fast | Better | ~1 GB |
| small | 244M | Medium | Good | ~2 GB |
| medium | 769M | Slow | Great | ~5 GB |
| large-v3 | 1.5B | Slowest | Best | ~10 GB |

---

## Basic Implementation

### Whisper Transcription Node

```python
#!/usr/bin/env python3
# whisper_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import sounddevice as sd
import queue
import threading

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_transcription')

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration', 3.0)

        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper {model_size} model...')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Model loaded')

        # Audio buffer
        self.audio_queue = queue.Queue()
        self.is_recording = False

        # Publishers
        self.transcription_pub = self.create_publisher(
            String, '/speech/transcription', 10)

        self.status_pub = self.create_publisher(
            String, '/speech/status', 10)

        # Start audio capture thread
        self.audio_thread = threading.Thread(target=self.audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

        # Processing timer
        self.timer = self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Whisper node ready')

    def audio_callback(self, indata, frames, time, status):
        """Callback for audio stream."""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(indata.copy())

    def audio_capture_loop(self):
        """Continuous audio capture."""
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            callback=self.audio_callback,
            blocksize=int(self.sample_rate * 0.1)
        ):
            while True:
                sd.sleep(100)

    def process_audio(self):
        """Process accumulated audio."""
        if self.audio_queue.qsize() < int(self.chunk_duration * 10):
            return

        # Collect audio chunks
        audio_data = []
        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())

        if not audio_data:
            return

        # Concatenate and process
        audio = np.concatenate(audio_data).flatten()

        # Voice Activity Detection (simple energy-based)
        energy = np.mean(audio ** 2)
        if energy < 0.001:  # Silence threshold
            return

        self.publish_status('processing')

        # Transcribe
        result = self.model.transcribe(
            audio,
            language=self.language,
            fp16=False
        )

        text = result['text'].strip()

        if text:
            self.get_logger().info(f'Transcribed: "{text}"')
            msg = String()
            msg.data = text
            self.transcription_pub.publish(msg)

        self.publish_status('listening')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Wake Word Detection

### Implementing Wake Word

```python
#!/usr/bin/env python3
# wake_word_detector.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import numpy as np

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')

        self.declare_parameter('wake_words', ['robot', 'hey robot', 'okay robot'])
        self.declare_parameter('timeout', 10.0)

        self.wake_words = self.get_parameter('wake_words').value
        self.timeout = self.get_parameter('timeout').value

        self.is_listening = False
        self.last_wake_time = None

        # Subscribers
        self.transcription_sub = self.create_subscription(
            String, '/speech/transcription',
            self.transcription_callback, 10)

        # Publishers
        self.command_pub = self.create_publisher(
            String, '/voice/command', 10)

        self.wake_pub = self.create_publisher(
            Bool, '/voice/wake_detected', 10)

        # Timeout timer
        self.timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info(f'Wake words: {self.wake_words}')

    def transcription_callback(self, msg):
        text = msg.data.lower()

        # Check for wake word
        wake_detected = any(wake in text for wake in self.wake_words)

        if wake_detected:
            self.is_listening = True
            self.last_wake_time = self.get_clock().now()

            # Publish wake detection
            wake_msg = Bool()
            wake_msg.data = True
            self.wake_pub.publish(wake_msg)

            # Extract command after wake word
            for wake in self.wake_words:
                if wake in text:
                    command = text.split(wake, 1)[-1].strip()
                    if command:
                        self.publish_command(command)
                    break

            self.get_logger().info('Wake word detected, listening...')

        elif self.is_listening:
            # Already listening, treat as command
            self.publish_command(text)

    def publish_command(self, command):
        if command:
            msg = String()
            msg.data = command
            self.command_pub.publish(msg)
            self.get_logger().info(f'Command: "{command}"')

    def check_timeout(self):
        if self.is_listening and self.last_wake_time:
            elapsed = (self.get_clock().now() - self.last_wake_time).nanoseconds / 1e9
            if elapsed > self.timeout:
                self.is_listening = False
                self.get_logger().info('Listening timeout, returning to idle')

def main(args=None):
    rclpy.init(args=args)
    node = WakeWordDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Command Parser

### Natural Language to Robot Commands

```python
#!/usr/bin/env python3
# command_parser.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re

class CommandParser(Node):
    def __init__(self):
        super().__init__('command_parser')

        # Command patterns
        self.patterns = {
            'navigate': [
                r'go to (?:the )?(.+)',
                r'navigate to (?:the )?(.+)',
                r'move to (?:the )?(.+)',
                r'walk to (?:the )?(.+)',
            ],
            'pick': [
                r'pick up (?:the )?(.+)',
                r'grab (?:the )?(.+)',
                r'get (?:the )?(.+)',
                r'take (?:the )?(.+)',
            ],
            'place': [
                r'put (?:it |that )?(?:on |in )?(?:the )?(.+)',
                r'place (?:it |that )?(?:on |in )?(?:the )?(.+)',
            ],
            'speak': [
                r'say (.+)',
                r'tell (?:me |them )?(.+)',
            ],
            'stop': [
                r'stop',
                r'halt',
                r'freeze',
            ],
            'status': [
                r'what(?:\'s| is) your status',
                r'how are you',
                r'status report',
            ],
        }

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/voice/command',
            self.command_callback, 10)

        # Publishers
        self.parsed_pub = self.create_publisher(
            String, '/robot/command', 10)

        self.feedback_pub = self.create_publisher(
            String, '/speech/feedback', 10)

    def command_callback(self, msg):
        text = msg.data.lower().strip()
        parsed = self.parse_command(text)

        if parsed:
            # Publish parsed command
            cmd_msg = String()
            cmd_msg.data = json.dumps(parsed)
            self.parsed_pub.publish(cmd_msg)

            # Provide feedback
            self.provide_feedback(parsed)
        else:
            self.get_logger().warn(f'Could not parse: "{text}"')
            self.speak("I didn't understand that command")

    def parse_command(self, text):
        """Parse natural language into structured command."""
        for action, patterns in self.patterns.items():
            for pattern in patterns:
                match = re.match(pattern, text, re.IGNORECASE)
                if match:
                    if match.groups():
                        target = match.group(1)
                        return {
                            'action': action,
                            'target': target,
                            'raw': text
                        }
                    else:
                        return {
                            'action': action,
                            'raw': text
                        }
        return None

    def provide_feedback(self, command):
        """Speak confirmation of understood command."""
        action = command['action']
        target = command.get('target', '')

        feedback_map = {
            'navigate': f"Going to {target}",
            'pick': f"Picking up {target}",
            'place': f"Placing on {target}",
            'stop': "Stopping",
            'status': "Checking status",
        }

        feedback = feedback_map.get(action, f"Executing {action}")
        self.speak(feedback)

    def speak(self, text):
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Text-to-Speech Response

### TTS Node

```python
#!/usr/bin/env python3
# tts_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import queue

class TTSNode(Node):
    def __init__(self):
        super().__init__('text_to_speech')

        self.declare_parameter('rate', 150)
        self.declare_parameter('volume', 0.9)

        # Initialize TTS engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', self.get_parameter('rate').value)
        self.engine.setProperty('volume', self.get_parameter('volume').value)

        # Speech queue
        self.speech_queue = queue.Queue()

        # Subscriber
        self.sub = self.create_subscription(
            String, '/speech/say',
            self.say_callback, 10)

        # Start speech thread
        self.speech_thread = threading.Thread(target=self.speech_loop)
        self.speech_thread.daemon = True
        self.speech_thread.start()

        self.get_logger().info('TTS node ready')

    def say_callback(self, msg):
        self.speech_queue.put(msg.data)

    def speech_loop(self):
        while True:
            text = self.speech_queue.get()
            self.get_logger().info(f'Speaking: "{text}"')
            self.engine.say(text)
            self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Complete Voice Pipeline Launch

```python
# launch/voice_pipeline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_voice',
            executable='whisper_node',
            name='whisper',
            parameters=[{
                'model_size': 'base',
                'language': 'en',
                'sample_rate': 16000,
            }],
            output='screen'
        ),
        Node(
            package='humanoid_voice',
            executable='wake_word_detector',
            name='wake_detector',
            parameters=[{
                'wake_words': ['robot', 'hey robot'],
                'timeout': 10.0,
            }],
            output='screen'
        ),
        Node(
            package='humanoid_voice',
            executable='command_parser',
            name='parser',
            output='screen'
        ),
        Node(
            package='humanoid_voice',
            executable='tts_node',
            name='tts',
            parameters=[{
                'rate': 150,
                'volume': 0.9,
            }],
            output='screen'
        ),
    ])
```

---

## Summary

This chapter covered voice-to-action implementation:

*   **Whisper ASR**: Speech recognition setup and configuration
*   **Wake word detection**: Activating listening mode
*   **Command parsing**: Natural language to structured commands
*   **Text-to-speech**: Robot verbal responses
*   **Integration**: Complete voice pipeline

Voice interfaces enable natural human-robot interaction. In the next chapter, we will explore LLM-based planning for more complex reasoning.
