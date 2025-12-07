# Chapter 24: Voice Command Implementation

This chapter implements voice command capabilities for the capstone humanoid assistant, building on the VLA concepts from Module 4.

---

## Voice Command System

```python
#!/usr/bin/env python3
# capstone_voice_system.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import threading
import queue

class CapstoneVoiceSystem(Node):
    def __init__(self):
        super().__init__('capstone_voice')

        # Load Whisper
        self.model = whisper.load_model("base")

        # Audio settings
        self.sample_rate = 16000
        self.chunk_duration = 3.0
        self.audio_queue = queue.Queue()

        # Wake word
        self.wake_words = ['assistant', 'robot', 'hey robot']
        self.is_listening = False
        self.listen_timeout = 10.0

        # Publishers
        self.command_pub = self.create_publisher(String, '/voice/command', 10)
        self.status_pub = self.create_publisher(String, '/voice/status', 10)
        self.feedback_pub = self.create_publisher(String, '/speech/say', 10)

        # Start audio thread
        self.audio_thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.audio_thread.start()

        # Processing timer
        self.timer = self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Voice system ready')

    def audio_callback(self, indata, frames, time, status):
        self.audio_queue.put(indata.copy())

    def audio_loop(self):
        with sd.InputStream(samplerate=self.sample_rate, channels=1,
                           dtype='float32', callback=self.audio_callback,
                           blocksize=int(self.sample_rate * 0.1)):
            while True:
                sd.sleep(100)

    def process_audio(self):
        if self.audio_queue.qsize() < int(self.chunk_duration * 10):
            return

        # Collect audio
        audio_chunks = []
        while not self.audio_queue.empty():
            audio_chunks.append(self.audio_queue.get())

        audio = np.concatenate(audio_chunks).flatten()

        # Voice activity detection
        if np.mean(audio ** 2) < 0.001:
            return

        # Transcribe
        result = self.model.transcribe(audio, language='en', fp16=False)
        text = result['text'].strip().lower()

        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        # Check for wake word
        if any(wake in text for wake in self.wake_words):
            self.is_listening = True
            self.speak("Yes, I'm listening")

            # Extract command after wake word
            for wake in self.wake_words:
                if wake in text:
                    cmd = text.split(wake, 1)[-1].strip()
                    if cmd:
                        self.process_command(cmd)
                    break

        elif self.is_listening:
            self.process_command(text)

    def process_command(self, command):
        self.get_logger().info(f'Command: "{command}"')

        # Publish command
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        # Acknowledge
        self.speak(f"Understood, {command}")

    def speak(self, text):
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CapstoneVoiceSystem()
    rclpy.spin(node)
```

---

## Command Parser

```python
#!/usr/bin/env python3
# capstone_command_parser.py

import re

class CommandParser(Node):
    def __init__(self):
        super().__init__('command_parser')

        self.command_patterns = {
            'navigate': [
                r'go to (?:the )?(.+)',
                r'walk to (?:the )?(.+)',
                r'navigate to (?:the )?(.+)',
            ],
            'fetch': [
                r'(?:fetch|bring|get) (?:me )?(?:the )?(.+)',
                r'grab (?:the )?(.+)',
            ],
            'status': [
                r"what'?s your (?:status|battery)",
                r'how are you',
            ],
            'stop': [
                r'stop',
                r'halt',
                r'wait',
            ],
            'follow': [
                r'follow me',
                r'come with me',
            ],
        }

        self.sub = self.create_subscription(
            String, '/voice/command', self.parse_callback, 10)
        self.pub = self.create_publisher(String, '/robot/parsed_command', 10)

    def parse_callback(self, msg):
        command = msg.data.lower()
        parsed = self.parse(command)

        if parsed:
            out = String()
            out.data = json.dumps(parsed)
            self.pub.publish(out)

    def parse(self, text):
        for action, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.match(pattern, text, re.IGNORECASE)
                if match:
                    return {
                        'action': action,
                        'target': match.group(1) if match.groups() else None,
                        'raw': text
                    }
        return {'action': 'unknown', 'raw': text}
```

---

## Text-to-Speech

```python
#!/usr/bin/env python3
# capstone_tts.py

import pyttsx3
import threading
import queue

class CapstoneTTS(Node):
    def __init__(self):
        super().__init__('capstone_tts')

        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)

        self.speech_queue = queue.Queue()

        self.sub = self.create_subscription(
            String, '/speech/say', self.say_callback, 10)

        self.speech_thread = threading.Thread(target=self.speech_loop, daemon=True)
        self.speech_thread.start()

    def say_callback(self, msg):
        self.speech_queue.put(msg.data)

    def speech_loop(self):
        while True:
            text = self.speech_queue.get()
            self.engine.say(text)
            self.engine.runAndWait()
```

---

## Launch File

```python
# launch/voice.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='humanoid_capstone', executable='voice_system'),
        Node(package='humanoid_capstone', executable='command_parser'),
        Node(package='humanoid_capstone', executable='tts'),
    ])
```

---

## Summary

This chapter implemented voice commands for the capstone:

*   Wake word detection
*   Speech-to-text with Whisper
*   Command parsing
*   Text-to-speech feedback

In the next chapter, we will implement path planning and navigation.
