# Chapter 15: Synthetic Data Generation

Synthetic data generation enables training perception models without collecting real-world data. This chapter covers using NVIDIA Isaac Sim's Replicator for generating diverse, annotated datasets for humanoid robotics applications.

---

## Why Synthetic Data?

| Advantage | Description |
|-----------|-------------|
| **Scalability** | Generate millions of images automatically |
| **Perfect labels** | Ground truth annotations without manual labeling |
| **Safety** | Train on dangerous scenarios without risk |
| **Diversity** | Domain randomization for robustness |
| **Cost** | Significantly cheaper than real data collection |

### Use Cases for Humanoids

*   Object detection and grasping
*   Human pose estimation
*   Scene understanding
*   Navigation obstacle detection
*   Hand-eye coordination

---

## Replicator Basics

### Setting Up Replicator

```python
import omni.replicator.core as rep
from omni.isaac.core import World

# Initialize world
world = World()
world.scene.add_default_ground_plane()

# Create writer for output
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/output/synthetic_data",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    distance_to_camera=True
)

# Attach writer to render product
render_product = rep.create.render_product("/World/Camera", (1280, 720))
writer.attach([render_product])
```

### Basic Data Generation Loop

```python
# Run replicator
with rep.trigger.on_frame(num_frames=1000):
    # Randomizations will be applied each frame
    rep.orchestrator.run()
```

---

## Domain Randomization

### Lighting Randomization

```python
def randomize_lighting():
    lights = rep.create.light(
        light_type="Sphere",
        position=rep.distribution.uniform((-5, -5, 3), (5, 5, 6)),
        intensity=rep.distribution.uniform(1000, 5000),
        color=rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0)),
        count=3
    )
    return lights

with rep.trigger.on_frame():
    randomize_lighting()
```

### Material Randomization

```python
def randomize_materials():
    # Get all meshes in scene
    meshes = rep.get.prims(path_pattern="/World/Objects/*")

    with meshes:
        rep.randomizer.materials(
            materials=[
                "omniverse://localhost/NVIDIA/Materials/Base/Metal/Aluminum.mdl",
                "omniverse://localhost/NVIDIA/Materials/Base/Plastic/Plastic_ABS.mdl",
                "omniverse://localhost/NVIDIA/Materials/Base/Wood/Oak.mdl",
            ]
        )
```

### Object Pose Randomization

```python
def randomize_objects():
    objects = rep.get.prims(semantics=[("class", "graspable")])

    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (0.3, -0.3, 0.8),  # Min bounds
                (0.7, 0.3, 1.0)    # Max bounds
            ),
            rotation=rep.distribution.uniform(
                (0, 0, 0),
                (0, 0, 360)
            )
        )
```

### Camera Pose Randomization

```python
def randomize_camera():
    camera = rep.get.prims(path_pattern="/World/Camera")

    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (-0.5, -0.5, 1.0),
                (0.5, 0.5, 1.5)
            ),
            look_at="/World/Table"
        )
```

---

## Annotation Types

### Bounding Boxes

```python
# Enable 2D bounding box output
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/output/bbox_data",
    bounding_box_2d_tight=True,
    bounding_box_2d_loose=True,
    bounding_box_3d=True
)
```

### Semantic Segmentation

```python
# Add semantic labels to objects
from omni.isaac.core.utils.semantics import add_update_semantics

# Label a cup as "graspable"
add_update_semantics(
    prim=stage.GetPrimAtPath("/World/Cup"),
    semantic_label="cup",
    type_label="class"
)

# Enable segmentation output
writer.initialize(
    output_dir="/output/segmentation",
    semantic_segmentation=True,
    colorize_semantic_segmentation=True
)
```

### Depth Maps

```python
writer.initialize(
    output_dir="/output/depth",
    distance_to_camera=True,
    distance_to_image_plane=True,
    normals=True
)
```

### Skeleton/Pose Annotations

```python
# For human pose estimation training
writer.initialize(
    output_dir="/output/pose",
    skeleton_data=True,
    bounding_box_2d_tight=True
)
```

---

## Complete Dataset Generation Pipeline

```python
# synthetic_data_pipeline.py
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Setup
world = World()
world.scene.add_default_ground_plane()

# Load environment
add_reference_to_stage(
    usd_path="/environments/kitchen.usd",
    prim_path="/World/Environment"
)

# Load objects to detect
objects_to_spawn = [
    ("cup", "/objects/cup.usd"),
    ("bottle", "/objects/bottle.usd"),
    ("bowl", "/objects/bowl.usd"),
]

# Spawn multiple instances
for obj_name, obj_path in objects_to_spawn:
    for i in range(5):
        add_reference_to_stage(
            usd_path=obj_path,
            prim_path=f"/World/Objects/{obj_name}_{i}"
        )
        # Add semantic label
        add_update_semantics(
            prim=stage.GetPrimAtPath(f"/World/Objects/{obj_name}_{i}"),
            semantic_label=obj_name,
            type_label="class"
        )

# Create camera
camera = rep.create.camera(
    position=(1.0, 0.0, 1.2),
    look_at=(0.5, 0.0, 0.8)
)

# Setup writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/output/object_detection_dataset",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    distance_to_camera=True
)

render_product = rep.create.render_product(camera, (1280, 720))
writer.attach([render_product])

# Define randomization graph
with rep.trigger.on_frame(num_frames=10000):
    # Randomize lighting
    rep.randomizer.light(
        light_type="Sphere",
        position=rep.distribution.uniform((-3, -3, 2), (3, 3, 4)),
        intensity=rep.distribution.uniform(500, 3000),
        count=rep.distribution.choice([2, 3, 4])
    )

    # Randomize object poses
    objects = rep.get.prims(semantics=[("class", "cup"), ("class", "bottle"), ("class", "bowl")])
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((0.2, -0.4, 0.8), (0.8, 0.4, 0.85)),
            rotation=rep.distribution.uniform((0, 0, 0), (30, 30, 360))
        )

    # Randomize camera
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((0.8, -0.3, 1.0), (1.2, 0.3, 1.4)),
            look_at="/World/Table"
        )

# Run generation
rep.orchestrator.run()
```

---

## Training Data Formats

### COCO Format Export

```python
from omni.replicator.core import AnnotatorRegistry

# Use COCO writer
coco_writer = rep.WriterRegistry.get("COCOWriter")
coco_writer.initialize(
    output_dir="/output/coco_format",
    semantic_types=["class"]
)
coco_writer.attach([render_product])
```

### KITTI Format Export

```python
kitti_writer = rep.WriterRegistry.get("KittiWriter")
kitti_writer.initialize(
    output_dir="/output/kitti_format",
    bbox_height_threshold=10,
    fully_visible_threshold=0.5
)
```

### Custom Format

```python
import json
import numpy as np

class CustomWriter(rep.Writer):
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.frame_id = 0

    def write(self, data):
        # Extract annotations
        rgb = data.get("rgb")
        bbox_2d = data.get("bounding_box_2d_tight")
        semantic = data.get("semantic_segmentation")

        # Save image
        rgb_path = f"{self.output_dir}/images/{self.frame_id:06d}.png"
        # Save RGB...

        # Save annotations
        annotations = {
            "frame_id": self.frame_id,
            "objects": []
        }

        for obj_id, bbox in bbox_2d["data"].items():
            annotations["objects"].append({
                "class": bbox["semanticLabel"],
                "bbox": [bbox["x_min"], bbox["y_min"], bbox["x_max"], bbox["y_max"]],
                "occlusion": bbox["occlusionRatio"]
            })

        with open(f"{self.output_dir}/labels/{self.frame_id:06d}.json", "w") as f:
            json.dump(annotations, f)

        self.frame_id += 1

# Register custom writer
rep.WriterRegistry.register(CustomWriter)
```

---

## Domain Randomization Strategies

### Texture Randomization

```python
def randomize_textures():
    # Load texture library
    textures = [
        "omniverse://localhost/NVIDIA/Materials/Base/Fabric/Cotton.mdl",
        "omniverse://localhost/NVIDIA/Materials/Base/Concrete/Concrete.mdl",
        "omniverse://localhost/NVIDIA/Materials/Base/Carpet/Carpet.mdl",
    ]

    floor = rep.get.prims(path_pattern="/World/Floor")
    with floor:
        rep.randomizer.materials(materials=textures)
```

### Background Randomization

```python
def randomize_background():
    # Use HDRI for environment lighting
    hdri_files = [
        "/hdri/indoor_1.hdr",
        "/hdri/indoor_2.hdr",
        "/hdri/outdoor_1.hdr",
    ]

    rep.create.dome_light(
        texture=rep.distribution.choice(hdri_files),
        intensity=rep.distribution.uniform(500, 1500)
    )
```

### Distractors

```python
def add_distractors():
    # Add random objects as distractors
    distractor_shapes = ["cube", "sphere", "cylinder"]

    for i in range(10):
        rep.create.shape(
            shape=rep.distribution.choice(distractor_shapes),
            position=rep.distribution.uniform((-2, -2, 0), (2, 2, 1)),
            scale=rep.distribution.uniform(0.05, 0.2),
            semantics=[("class", "distractor")]
        )
```

---

## Best Practices

### 1. Start Simple

```python
# Begin with basic randomization
with rep.trigger.on_frame(num_frames=100):
    # Only position randomization first
    randomize_object_positions()
```

### 2. Validate Generated Data

```python
# Visually inspect samples
rep.orchestrator.preview()

# Check annotation quality
def validate_annotations(data):
    bbox = data.get("bounding_box_2d_tight")
    for obj in bbox["data"].values():
        if obj["x_max"] - obj["x_min"] < 10:
            print(f"Warning: Small bbox for {obj['semanticLabel']}")
```

### 3. Balance Dataset

```python
# Ensure class balance
class_counts = {"cup": 0, "bottle": 0, "bowl": 0}
target_per_class = 3000

while min(class_counts.values()) < target_per_class:
    # Generate frame with specific object focus
    # ...
    pass
```

---

## Summary

This chapter covered synthetic data generation:

*   **Replicator basics**: Writers, render products, triggers
*   **Domain randomization**: Lighting, materials, poses, cameras
*   **Annotation types**: Bounding boxes, segmentation, depth
*   **Data pipelines**: Complete generation workflows
*   **Export formats**: COCO, KITTI, custom formats
*   **Best practices**: Validation and balancing

Synthetic data accelerates perception model development for humanoid robots. In the next chapter, we will explore Isaac ROS for VSLAM and navigation.
