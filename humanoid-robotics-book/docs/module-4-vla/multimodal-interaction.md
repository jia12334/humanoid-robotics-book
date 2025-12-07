# Chapter 21: Multi-modal Human-Robot Interaction

Multi-modal interaction combines vision, language, and other modalities for natural human-robot communication. This chapter covers integrating multiple input channels for humanoid robots.

---

## Multi-Modal Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    Input Modalities                         │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────┐    │
│  │  Voice  │  │ Vision  │  │ Gesture │  │   Touch     │    │
│  │ Whisper │  │  YOLO   │  │ MediaPipe│  │  Sensors   │    │
│  └────┬────┘  └────┬────┘  └────┬────┘  └──────┬──────┘    │
└───────┼────────────┼────────────┼──────────────┼────────────┘
        │            │            │              │
        ▼            ▼            ▼              ▼
┌─────────────────────────────────────────────────────────────┐
│                  Fusion Module                              │
│           Context + Intent Understanding                    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   Action Generation                         │
└─────────────────────────────────────────────────────────────┘
```

---

## Vision-Language Models

### Using CLIP for Object Grounding

```python
#!/usr/bin/env python3
# clip_grounding.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import clip
from PIL import Image as PILImage
import numpy as np

class CLIPGroundingNode(Node):
    def __init__(self):
        super().__init__('clip_grounding')

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)
        self.bridge = CvBridge()

        self.current_image = None

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.query_sub = self.create_subscription(
            String, '/vision/query', self.query_callback, 10)

        self.result_pub = self.create_publisher(
            String, '/vision/grounding_result', 10)

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def query_callback(self, msg):
        if self.current_image is None:
            return

        query = msg.data
        result = self.ground_object(query)

        out = String()
        out.data = json.dumps(result)
        self.result_pub.publish(out)

    def ground_object(self, query):
        """Find object matching natural language query."""
        # Prepare image
        pil_image = PILImage.fromarray(self.current_image)
        image_input = self.preprocess(pil_image).unsqueeze(0).to(self.device)

        # Prepare text
        text_input = clip.tokenize([query]).to(self.device)

        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_input)

            similarity = (image_features @ text_features.T).softmax(dim=-1)
            confidence = similarity[0][0].item()

        return {
            'query': query,
            'confidence': confidence,
            'found': confidence > 0.5
        }
```

---

## Gesture Recognition

### MediaPipe Gesture Detection

```python
#!/usr/bin/env python3
# gesture_recognition.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import mediapipe as mp
import cv2
import json

class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition')

        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7
        )

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.gesture_pub = self.create_publisher(
            String, '/gesture/detected', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        results = self.hands.process(cv_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                gesture = self.classify_gesture(hand_landmarks)
                if gesture:
                    out = String()
                    out.data = json.dumps(gesture)
                    self.gesture_pub.publish(out)

    def classify_gesture(self, landmarks):
        """Classify hand gesture from landmarks."""
        # Get fingertip positions
        thumb_tip = landmarks.landmark[4]
        index_tip = landmarks.landmark[8]
        middle_tip = landmarks.landmark[12]
        ring_tip = landmarks.landmark[16]
        pinky_tip = landmarks.landmark[20]

        # Simple gesture classification
        fingers_up = self.count_fingers_up(landmarks)

        gesture_map = {
            0: 'fist',
            1: 'pointing',
            2: 'peace',
            5: 'open_hand',
        }

        gesture = gesture_map.get(fingers_up)
        if gesture:
            return {
                'gesture': gesture,
                'fingers_up': fingers_up,
                'position': {
                    'x': index_tip.x,
                    'y': index_tip.y
                }
            }
        return None

    def count_fingers_up(self, landmarks):
        tips = [4, 8, 12, 16, 20]
        pips = [3, 6, 10, 14, 18]
        count = 0

        for tip, pip in zip(tips, pips):
            if landmarks.landmark[tip].y < landmarks.landmark[pip].y:
                count += 1

        return count
```

---

## Multi-Modal Fusion

```python
#!/usr/bin/env python3
# multimodal_fusion.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import deque
import time

class MultiModalFusion(Node):
    def __init__(self):
        super().__init__('multimodal_fusion')

        # Input buffers with timestamps
        self.voice_buffer = deque(maxlen=5)
        self.gesture_buffer = deque(maxlen=10)
        self.vision_buffer = deque(maxlen=5)

        # Subscribers
        self.voice_sub = self.create_subscription(
            String, '/voice/command', self.voice_callback, 10)
        self.gesture_sub = self.create_subscription(
            String, '/gesture/detected', self.gesture_callback, 10)
        self.vision_sub = self.create_subscription(
            String, '/vision/objects', self.vision_callback, 10)

        # Publisher
        self.intent_pub = self.create_publisher(
            String, '/robot/intent', 10)

        # Fusion timer
        self.timer = self.create_timer(0.1, self.fuse_inputs)

    def voice_callback(self, msg):
        self.voice_buffer.append({
            'time': time.time(),
            'data': msg.data
        })

    def gesture_callback(self, msg):
        self.gesture_buffer.append({
            'time': time.time(),
            'data': json.loads(msg.data)
        })

    def vision_callback(self, msg):
        self.vision_buffer.append({
            'time': time.time(),
            'data': json.loads(msg.data)
        })

    def fuse_inputs(self):
        """Combine multi-modal inputs into unified intent."""
        current_time = time.time()
        window = 2.0  # 2 second fusion window

        # Get recent inputs
        recent_voice = [v for v in self.voice_buffer
                       if current_time - v['time'] < window]
        recent_gestures = [g for g in self.gesture_buffer
                         if current_time - g['time'] < window]
        recent_vision = [v for v in self.vision_buffer
                        if current_time - v['time'] < window]

        if not recent_voice and not recent_gestures:
            return

        intent = self.interpret_intent(
            recent_voice, recent_gestures, recent_vision)

        if intent:
            out = String()
            out.data = json.dumps(intent)
            self.intent_pub.publish(out)

    def interpret_intent(self, voice, gestures, vision):
        """Interpret user intent from multi-modal inputs."""
        intent = {'confidence': 0.0}

        # Voice provides main command
        if voice:
            voice_cmd = voice[-1]['data'].lower()
            intent['voice_command'] = voice_cmd

            # "Pick up that" + pointing gesture
            if 'that' in voice_cmd or 'this' in voice_cmd:
                if gestures:
                    latest_gesture = gestures[-1]['data']
                    if latest_gesture['gesture'] == 'pointing':
                        # Use pointing direction + visible objects
                        if vision:
                            pointed_object = self.find_pointed_object(
                                latest_gesture, vision[-1]['data'])
                            if pointed_object:
                                intent['target'] = pointed_object
                                intent['confidence'] = 0.9

            # Direct object reference
            elif vision:
                for obj in vision[-1]['data']:
                    if obj['label'].lower() in voice_cmd:
                        intent['target'] = obj
                        intent['confidence'] = 0.8
                        break

        # Gesture-only commands
        elif gestures:
            gesture = gestures[-1]['data']
            if gesture['gesture'] == 'open_hand':
                intent['command'] = 'stop'
                intent['confidence'] = 0.7
            elif gesture['gesture'] == 'pointing':
                intent['command'] = 'look_at'
                intent['direction'] = gesture['position']
                intent['confidence'] = 0.6

        return intent if intent['confidence'] > 0.5 else None

    def find_pointed_object(self, gesture, objects):
        """Find object closest to pointing direction."""
        point_x = gesture['position']['x']
        point_y = gesture['position']['y']

        closest = None
        min_dist = float('inf')

        for obj in objects:
            obj_x = (obj['bbox'][0] + obj['bbox'][2]) / 2
            obj_y = (obj['bbox'][1] + obj['bbox'][3]) / 2
            dist = ((point_x - obj_x)**2 + (point_y - obj_y)**2)**0.5

            if dist < min_dist:
                min_dist = dist
                closest = obj

        return closest
```

---

## Gaze Tracking

```python
class GazeTracker(Node):
    def __init__(self):
        super().__init__('gaze_tracker')

        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True
        )

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.gaze_pub = self.create_publisher(
            String, '/user/gaze', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        results = self.face_mesh.process(cv_image)

        if results.multi_face_landmarks:
            landmarks = results.multi_face_landmarks[0]
            gaze = self.estimate_gaze(landmarks)

            out = String()
            out.data = json.dumps(gaze)
            self.gaze_pub.publish(out)

    def estimate_gaze(self, landmarks):
        # Eye landmarks for gaze estimation
        left_eye = [landmarks.landmark[i] for i in [33, 133, 160, 159, 158, 144, 145, 153]]
        right_eye = [landmarks.landmark[i] for i in [362, 263, 387, 386, 385, 373, 374, 380]]

        # Simplified gaze direction
        left_center = np.mean([[l.x, l.y] for l in left_eye], axis=0)
        right_center = np.mean([[l.x, l.y] for l in right_eye], axis=0)
        gaze_center = (left_center + right_center) / 2

        return {
            'x': float(gaze_center[0]),
            'y': float(gaze_center[1]),
            'looking_at_robot': abs(gaze_center[0] - 0.5) < 0.2
        }
```

---

## Summary

This chapter covered multi-modal interaction:

*   **Vision-Language**: CLIP for object grounding
*   **Gesture recognition**: MediaPipe hand tracking
*   **Multi-modal fusion**: Combining inputs
*   **Gaze tracking**: Understanding user attention

Multi-modal interfaces enable natural, intuitive robot interaction. In the next chapter, we will explore complete VLA pipeline examples.
