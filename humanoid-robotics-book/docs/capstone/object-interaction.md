# Chapter 26: Object Detection & Interaction

This chapter implements object detection and manipulation for the capstone humanoid assistant.

---

## Object Detection

```python
#!/usr/bin/env python3
# capstone_detection.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json

class CapstoneDetection(Node):
    def __init__(self):
        super().__init__('capstone_detection')

        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        self.detected_objects = {}

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.objects_pub = self.create_publisher(String, '/detection/objects', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)

        objects = []
        for r in results:
            for box in r.boxes:
                obj = {
                    'label': self.model.names[int(box.cls)],
                    'confidence': float(box.conf),
                    'bbox': box.xyxy[0].tolist(),
                    'center': {
                        'x': float((box.xyxy[0][0] + box.xyxy[0][2]) / 2),
                        'y': float((box.xyxy[0][1] + box.xyxy[0][3]) / 2),
                    }
                }
                objects.append(obj)

        # Update detected objects
        for obj in objects:
            self.detected_objects[obj['label']] = obj

        # Publish
        msg = String()
        msg.data = json.dumps(objects)
        self.objects_pub.publish(msg)
```

---

## Manipulation Controller

```python
#!/usr/bin/env python3
# capstone_manipulation.py

class CapstoneManipulation(Node):
    def __init__(self):
        super().__init__('capstone_manipulation')

        self.detected_objects = {}
        self.held_object = None

        self.detection_sub = self.create_subscription(
            String, '/detection/objects', self.detection_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/robot/parsed_command', self.command_callback, 10)

        self.arm_pub = self.create_publisher(String, '/arm/command', 10)
        self.speech_pub = self.create_publisher(String, '/speech/say', 10)

    def detection_callback(self, msg):
        objects = json.loads(msg.data)
        for obj in objects:
            self.detected_objects[obj['label']] = obj

    def command_callback(self, msg):
        command = json.loads(msg.data)

        if command['action'] == 'fetch':
            self.fetch_object(command['target'])

    def fetch_object(self, target):
        # Find object
        obj = None
        for label, detected in self.detected_objects.items():
            if target.lower() in label.lower():
                obj = detected
                break

        if not obj:
            self.speak(f"I can't see {target}")
            return

        self.speak(f"Picking up {target}")

        # Send arm command
        arm_cmd = {
            'action': 'pick',
            'target': obj['center'],
            'label': obj['label']
        }
        msg = String()
        msg.data = json.dumps(arm_cmd)
        self.arm_pub.publish(msg)

        self.held_object = target

    def speak(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
```

---

## Summary

This chapter implemented object interaction:

*   YOLOv8 object detection
*   Object tracking and localization
*   Pick and place commands
*   Arm control interface

In the next chapter, we will bring everything together in the end-to-end demo.
