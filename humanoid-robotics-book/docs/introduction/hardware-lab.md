# Chapter 4: Hardware Requirements & Lab Setup

This chapter outlines the hardware requirements and lab setup needed to follow along with the practical exercises in this book. While many concepts can be learned through simulation, having access to physical hardware will significantly enhance your understanding of Physical AI and humanoid robotics.

---

## Minimum System Requirements

To run the software stack covered in this book, your development machine should meet the following specifications:

### Development Workstation

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Intel Core i5 (8th gen) / AMD Ryzen 5 | Intel Core i7/i9 (12th gen+) / AMD Ryzen 7/9 |
| **RAM** | 16 GB | 32 GB or more |
| **GPU** | NVIDIA GTX 1060 (6GB VRAM) | NVIDIA RTX 3070/4070 or higher (8GB+ VRAM) |
| **Storage** | 256 GB SSD | 512 GB NVMe SSD or larger |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS (native or dual-boot) |

### Why NVIDIA GPUs?

NVIDIA GPUs are strongly recommended for this curriculum due to:

*   **CUDA Support**: Essential for running NVIDIA Isaac Sim and deep learning frameworks.
*   **Isaac Sim Compatibility**: NVIDIA Isaac Sim requires RTX-capable GPUs for real-time ray tracing and physics simulation.
*   **Deep Learning Acceleration**: Training and inference for perception models benefit greatly from CUDA cores and Tensor cores.

---

## Operating System Setup

### Ubuntu 22.04 LTS (Recommended)

ROS 2 Humble Hawksbill, the primary robotics framework used in this book, is best supported on Ubuntu 22.04 LTS. While Windows and macOS have partial ROS 2 support, we strongly recommend Ubuntu for the smoothest experience.

**Installation Options:**

1.  **Native Installation**: Best performance; recommended for serious development.
2.  **Dual Boot**: Run Ubuntu alongside Windows; good balance of convenience and performance.
3.  **Virtual Machine**: Use VMware or VirtualBox; suitable for initial exploration but may have GPU limitations.
4.  **WSL2 (Windows Subsystem for Linux)**: Viable for ROS 2 development but limited GPU passthrough for Isaac Sim.

### Essential Software Stack

After installing Ubuntu 22.04, you will need to install the following:

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential development tools
sudo apt install -y build-essential cmake git curl wget

# Install Python 3.10 (default on Ubuntu 22.04)
sudo apt install -y python3 python3-pip python3-venv

# Install ROS 2 Humble (see Module 1 for detailed instructions)
# Install Gazebo (see Module 2 for detailed instructions)
# Install NVIDIA Isaac Sim (see Module 3 for detailed instructions)
```

---

## Edge Computing Hardware

For deploying AI models on physical robots, edge computing platforms are essential. The following are recommended:

### NVIDIA Jetson Family

| Platform | Use Case | Key Specs |
|----------|----------|-----------|
| **Jetson Orin Nano** | Entry-level edge AI | 40 TOPS, 8GB RAM |
| **Jetson Orin NX** | Mid-range robotics | 100 TOPS, 16GB RAM |
| **Jetson AGX Orin** | High-performance humanoids | 275 TOPS, 32/64GB RAM |

**Why Jetson for Robotics?**

*   Native CUDA support for AI inference
*   ROS 2 and Isaac ROS compatibility
*   Low power consumption suitable for mobile robots
*   Extensive documentation and community support

### Setting Up a Jetson Device

1.  Download the latest JetPack SDK from NVIDIA Developer website.
2.  Flash the SD card or NVMe drive using the SDK Manager.
3.  Boot the Jetson and complete initial setup.
4.  Install ROS 2 and Isaac ROS packages.

---

## Sensors and Peripherals

Physical AI systems rely on sensors for perception. Here are the recommended sensors for hands-on learning:

### Vision Sensors

| Sensor | Type | Use Case | Approximate Cost |
|--------|------|----------|------------------|
| **Intel RealSense D435i** | RGB-D Camera | Depth perception, SLAM | $300-400 |
| **Intel RealSense D455** | RGB-D Camera | Longer range depth | $400-500 |
| **Stereolabs ZED 2i** | Stereo Camera | High-quality 3D mapping | $450-550 |
| **OAK-D Lite** | RGB-D + AI | On-device AI processing | $150-200 |

### LiDAR Sensors

| Sensor | Type | Range | Approximate Cost |
|--------|------|-------|------------------|
| **RPLiDAR A1** | 2D LiDAR | 12m | $100-150 |
| **RPLiDAR A2** | 2D LiDAR | 18m | $300-400 |
| **Livox Mid-360** | 3D LiDAR | 40m | $1000-1500 |

### IMU (Inertial Measurement Unit)

*   **BNO055**: 9-axis IMU, easy to integrate, ~$30
*   **ICM-20948**: High-performance 9-axis IMU, ~$15
*   **VectorNav VN-100**: Industrial-grade IMU, ~$1000+

### Audio Input

For voice-to-action implementations:

*   **ReSpeaker Mic Array**: Far-field voice capture, ~$80
*   **USB Conference Microphone**: General-purpose audio input, ~$50-100

---

## Recommended Robot Platforms

While this book focuses on humanoid robotics concepts applicable to any platform, the following robots are recommended for practical exercises:

### Educational and Research Humanoids

| Robot | Manufacturer | Use Case | Price Range |
|-------|--------------|----------|-------------|
| **Unitree H1** | Unitree | Full-size humanoid research | $90,000+ |
| **Unitree G1** | Unitree | Compact humanoid development | $16,000+ |
| **Fourier GR-1** | Fourier Intelligence | Research humanoid | Contact for pricing |
| **Figure 01/02** | Figure AI | Advanced humanoid (limited access) | Research partnership |

### Alternative Platforms for Learning

If full humanoid platforms are not accessible, these alternatives provide valuable learning experiences:

| Platform | Type | Learning Value | Price Range |
|----------|------|----------------|-------------|
| **Unitree Go2** | Quadruped | ROS 2, navigation, SLAM | $1,600-2,800 |
| **TurtleBot 4** | Mobile robot | ROS 2 fundamentals | $1,200-1,800 |
| **Open Manipulator-X** | Robot arm | Manipulation, MoveIt2 | $500-700 |
| **Niryo Ned2** | Cobot arm | Industrial applications | $3,000-5,000 |

### DIY and Simulation-First Approach

For learners without access to physical robots:

1.  **Simulation-First**: All exercises can be completed in Gazebo and Isaac Sim.
2.  **3D Printed Components**: Design and print custom robot parts.
3.  **Arduino/Servo Kits**: Build simple actuated systems for basic control concepts.

---

## Lab Environment Setup

### Physical Lab Requirements

If setting up a physical robotics lab:

*   **Space**: Minimum 3m x 3m clear floor area for robot testing
*   **Power**: Multiple outlets, UPS for sensitive equipment
*   **Safety**: Emergency stop buttons, safety barriers for larger robots
*   **Network**: Wired Ethernet for low-latency ROS 2 communication
*   **Lighting**: Consistent lighting for vision systems

### Virtual Lab Setup

For remote or simulation-based learning:

```text
Virtual Lab Architecture
├── Development VM (Ubuntu 22.04)
│   ├── ROS 2 Humble
│   ├── Gazebo Fortress
│   └── Development tools
├── Isaac Sim Workstation
│   ├── NVIDIA Omniverse
│   └── Isaac Sim
└── Version Control
    └── Git repository for code
```

---

## Software Installation Checklist

Use this checklist to ensure your environment is ready:

- [ ] Ubuntu 22.04 LTS installed
- [ ] NVIDIA drivers installed (if applicable)
- [ ] CUDA Toolkit installed (if applicable)
- [ ] ROS 2 Humble installed
- [ ] Gazebo Fortress/Garden installed
- [ ] Python 3.10 with pip and venv
- [ ] Git configured with SSH keys
- [ ] VS Code or preferred IDE installed
- [ ] Docker installed (optional, for containerized workflows)
- [ ] NVIDIA Isaac Sim installed (Module 3)

---

## Cloud and Remote Options

If local hardware is limited, consider these cloud options:

### Cloud GPU Instances

| Provider | Service | GPU Options | Use Case |
|----------|---------|-------------|----------|
| **AWS** | EC2 G4/G5 instances | T4, A10G | Isaac Sim, training |
| **Google Cloud** | Compute Engine | T4, V100, A100 | Training, simulation |
| **NVIDIA NGC** | DGX Cloud | A100, H100 | Enterprise workloads |
| **Lambda Labs** | GPU Cloud | A10, A100 | Cost-effective training |

### Remote Desktop Solutions

*   **NoMachine**: High-performance remote desktop
*   **Parsec**: Low-latency game streaming (works for robotics visualization)
*   **NICE DCV**: AWS-optimized remote desktop

---

## Budget Recommendations

### Minimal Setup (~$500-1,000)

*   Used workstation with GTX 1060+ GPU
*   RPLiDAR A1 or OAK-D Lite
*   Focus on simulation-based learning

### Intermediate Setup (~$2,000-5,000)

*   Workstation with RTX 3060/3070
*   Intel RealSense D435i
*   TurtleBot 4 or similar mobile robot
*   Jetson Orin Nano

### Advanced Setup (~$10,000+)

*   High-end workstation with RTX 4080/4090
*   Multiple depth cameras and LiDAR
*   Quadruped robot (Unitree Go2)
*   Jetson AGX Orin
*   Robot arm for manipulation

---

## Summary

Setting up a proper hardware and software environment is crucial for success in Physical AI and humanoid robotics. While simulation provides an excellent starting point, hands-on experience with physical hardware accelerates learning. Start with the minimum requirements and gradually expand your lab as you progress through the modules. Remember that many concepts can be mastered through simulation alone, making this field accessible even without expensive hardware.

In the next module, we will dive into ROS 2, the robotic nervous system that ties all these hardware components together into a cohesive robotics platform.
