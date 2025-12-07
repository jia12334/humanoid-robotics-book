# Chapter 14: NVIDIA Isaac Sim Overview

NVIDIA Isaac Sim is a robotics simulation platform built on NVIDIA Omniverse, providing photorealistic rendering, accurate physics, and AI-ready synthetic data generation. This chapter introduces Isaac Sim's capabilities for humanoid robotics development.

---

## What is Isaac Sim?

Isaac Sim provides:

*   **Photorealistic rendering**: RTX ray tracing for realistic visuals
*   **Accurate physics**: PhysX 5 with GPU acceleration
*   **Synthetic data generation**: Domain randomization for AI training
*   **ROS 2 integration**: Native bridge to robot software
*   **USD-based workflow**: Universal Scene Description for complex scenes

### Isaac Sim vs Traditional Simulators

| Feature | Isaac Sim | Gazebo | Unity |
|---------|-----------|--------|-------|
| **Rendering** | RTX ray tracing | Rasterization | Rasterization/HDRP |
| **Physics** | PhysX 5 (GPU) | DART/Bullet | PhysX |
| **Synthetic data** | Built-in | Manual | Manual |
| **ML integration** | Native | External | External |
| **GPU requirement** | RTX GPU required | Optional | Optional |

---

## System Requirements

### Minimum Requirements

| Component | Specification |
|-----------|---------------|
| **GPU** | NVIDIA RTX 2070 or higher |
| **VRAM** | 8 GB |
| **RAM** | 32 GB |
| **Storage** | 50 GB SSD |
| **OS** | Ubuntu 20.04/22.04, Windows 10/11 |

### Recommended for Humanoid Simulation

| Component | Specification |
|-----------|---------------|
| **GPU** | NVIDIA RTX 4080 or higher |
| **VRAM** | 16+ GB |
| **RAM** | 64 GB |
| **Storage** | 100 GB NVMe SSD |

---

## Installation

### Using NVIDIA Omniverse Launcher

1. Download Omniverse Launcher from [nvidia.com/omniverse](https://nvidia.com/omniverse)
2. Install Omniverse Launcher
3. Navigate to Exchange → Isaac Sim
4. Click Install

### Docker Installation

```bash
# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run with GPU support
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

---

## Isaac Sim Interface

### Main Components

```text
┌─────────────────────────────────────────────────────────────┐
│                    Isaac Sim Interface                      │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌────────────────────┐  ┌─────────────┐  │
│  │   Stage     │  │      Viewport      │  │  Property   │  │
│  │   Panel     │  │                    │  │    Panel    │  │
│  │             │  │   [3D Scene View]  │  │             │  │
│  │  - World    │  │                    │  │  Transform  │  │
│  │  - Robot    │  │                    │  │  Physics    │  │
│  │  - Sensors  │  │                    │  │  Materials  │  │
│  └─────────────┘  └────────────────────┘  └─────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────┐    │
│  │                    Timeline / Console               │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Key Menus

*   **Isaac Examples**: Pre-built robot demos and tutorials
*   **Isaac Utils**: Robot importers, joint configurators
*   **Isaac Sensors**: Camera, LiDAR, IMU setup
*   **Replicator**: Synthetic data generation tools

---

## Importing Humanoid Robots

### URDF Import

```python
# Python script for URDF import
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.urdf")

from omni.isaac.urdf import _urdf
import omni.kit.commands

# Configure import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.import_inertia_tensor = True
import_config.distance_scale = 1.0
import_config.density = 0.0
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength = 1000.0
import_config.default_position_drive_damping = 100.0

# Import URDF
result, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/path/to/humanoid.urdf",
    import_config=import_config
)
```

### Post-Import Configuration

```python
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

# Get robot articulation
robot = Articulation(prim_path="/World/humanoid")
robot.initialize()

# Configure joint drives
for joint_name in robot.dof_names:
    robot.set_joint_position_target(
        positions=[0.0],
        joint_indices=[robot.get_dof_index(joint_name)]
    )
```

---

## Physics Configuration

### PhysX Settings

```python
from omni.isaac.core import PhysicsContext

# Configure physics
physics_context = PhysicsContext()
physics_context.set_physics_dt(1.0 / 1000.0)  # 1000 Hz
physics_context.set_solver_type("TGS")  # Temporal Gauss-Seidel
physics_context.set_gravity([0.0, 0.0, -9.81])
```

### Articulation Configuration

```python
from pxr import UsdPhysics, PhysxSchema

# Get articulation prim
stage = omni.usd.get_context().get_stage()
robot_prim = stage.GetPrimAtPath("/World/humanoid")

# Configure articulation
articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
articulation_api.CreateEnabledSelfCollisionsAttr().Set(True)
articulation_api.CreateSolverPositionIterationCountAttr().Set(32)
articulation_api.CreateSolverVelocityIterationCountAttr().Set(16)
```

### Joint Drive Settings

```python
# Configure individual joint drives
joint_prim = stage.GetPrimAtPath("/World/humanoid/left_knee")
drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")

drive_api.CreateTypeAttr().Set("force")
drive_api.CreateStiffnessAttr().Set(10000.0)
drive_api.CreateDampingAttr().Set(1000.0)
drive_api.CreateMaxForceAttr().Set(1000.0)
```

---

## ROS 2 Integration

### Enabling ROS 2 Bridge

```python
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS 2 bridge
enable_extension("omni.isaac.ros2_bridge")
```

### Publishing Joint States

```python
import omni.graph.core as og

# Create OmniGraph for ROS publishing
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ROS_JointState", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("ArticulationController.inputs:robotPath", "/World/humanoid"),
            ("PublishJointState.inputs:targetPrim", "/World/humanoid"),
            ("PublishJointState.inputs:topicName", "/joint_states"),
        ],
    },
)
```

### Subscribing to Commands

```python
# Create subscriber for joint commands
og.Controller.edit(
    {"graph_path": "/World/ROS_JointCommand", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ],
        keys.SET_VALUES: [
            ("SubscribeJointState.inputs:topicName", "/joint_commands"),
            ("ArticulationController.inputs:robotPath", "/World/humanoid"),
        ],
    },
)
```

---

## Sensor Setup

### RGB-D Camera

```python
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

# Create camera
camera = Camera(
    prim_path="/World/humanoid/head/camera",
    position=[0.05, 0, 0.1],
    frequency=30,
    resolution=(640, 480)
)

# Add depth output
camera.add_distance_to_camera_to_frame()
camera.add_instance_segmentation_to_frame()
```

### LiDAR Sensor

```python
from omni.isaac.range_sensor import _range_sensor

# Create LiDAR
lidar_config = _range_sensor.LidarSensorCreateConfig()
lidar_config.name = "lidar"
lidar_config.position = [0, 0, 0.5]
lidar_config.orientation = [0, 0, 0, 1]
lidar_config.min_range = 0.1
lidar_config.max_range = 100.0
lidar_config.horizontal_fov = 360.0
lidar_config.horizontal_resolution = 0.2
lidar_config.vertical_fov = 30.0
lidar_config.vertical_resolution = 1.0

lidar = _range_sensor.LidarSensor(
    prim_path="/World/humanoid/lidar",
    config=lidar_config
)
```

### IMU Sensor

```python
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(
    prim_path="/World/humanoid/pelvis/imu",
    name="imu",
    frequency=200,
    linear_acceleration_filter_size=10,
    angular_velocity_filter_size=10
)
```

---

## Running Simulations

### Standalone Script

```python
# standalone_sim.py
from omni.isaac.kit import SimulationApp

# Initialize simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Import robot
add_reference_to_stage(
    usd_path="/path/to/humanoid.usd",
    prim_path="/World/humanoid"
)

# Reset world
world.reset()

# Simulation loop
while simulation_app.is_running():
    world.step(render=True)

    # Get robot state
    robot = world.scene.get_object("humanoid")
    joint_positions = robot.get_joint_positions()

    # Apply control
    robot.set_joint_position_targets(target_positions)

simulation_app.close()
```

### Extension-Based Application

```python
# extension.py
import omni.ext
from omni.isaac.core import World

class HumanoidSimExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._world = World()
        self._world.scene.add_default_ground_plane()
        # Setup robot, sensors, etc.

    def on_shutdown(self):
        self._world.clear()
```

---

## Summary

This chapter introduced NVIDIA Isaac Sim:

*   **Platform overview**: Capabilities and requirements
*   **Installation**: Omniverse Launcher and Docker
*   **Robot import**: URDF to USD workflow
*   **Physics configuration**: PhysX settings for humanoids
*   **ROS 2 integration**: OmniGraph-based bridges
*   **Sensor setup**: Cameras, LiDAR, IMU

Isaac Sim provides the most advanced simulation capabilities for developing AI-powered humanoid robots. In the next chapter, we will explore synthetic data generation for training perception models.
