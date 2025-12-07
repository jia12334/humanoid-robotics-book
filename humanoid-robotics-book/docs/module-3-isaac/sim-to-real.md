# Chapter 18: Sim-to-Real Transfer Techniques

Sim-to-real transfer bridges the gap between simulated training and real-world deployment. This chapter covers techniques for ensuring policies and models trained in simulation work effectively on physical humanoid robots.

---

## The Sim-to-Real Gap

### Sources of Discrepancy

| Domain | Simulation | Reality |
|--------|------------|---------|
| **Physics** | Idealized models | Complex dynamics |
| **Sensors** | Perfect data | Noise, latency |
| **Actuators** | Instant response | Delays, backlash |
| **Environment** | Controlled | Unpredictable |
| **Visuals** | Rendered | Real lighting |

### Transfer Challenges for Humanoids

*   Balance control sensitivity
*   Ground contact dynamics
*   Joint flexibility and compliance
*   Sensor noise characteristics
*   Communication latencies

---

## Domain Randomization

### Physics Randomization

```python
# physics_randomization.py
import numpy as np
from omni.isaac.core.prims import RigidPrim

class PhysicsRandomizer:
    def __init__(self, robot_prim_path):
        self.robot_path = robot_prim_path

    def randomize_mass(self, link_name, nominal_mass, variation=0.1):
        """Randomize link mass within ±variation%."""
        randomized_mass = nominal_mass * np.random.uniform(
            1 - variation, 1 + variation
        )
        # Apply to USD prim
        link_prim = RigidPrim(f"{self.robot_path}/{link_name}")
        link_prim.set_mass(randomized_mass)

    def randomize_friction(self, nominal_friction=1.0, variation=0.3):
        """Randomize ground friction."""
        friction = nominal_friction * np.random.uniform(
            1 - variation, 1 + variation
        )
        # Apply to ground plane
        return friction

    def randomize_joint_dynamics(self, joint_name, nominal_damping, variation=0.2):
        """Randomize joint damping."""
        damping = nominal_damping * np.random.uniform(
            1 - variation, 1 + variation
        )
        return damping

    def apply_randomization(self):
        """Apply all randomizations."""
        # Mass randomization
        link_masses = {
            'torso': 15.0,
            'left_thigh': 3.5,
            'right_thigh': 3.5,
            'left_shin': 2.5,
            'right_shin': 2.5,
        }
        for link, mass in link_masses.items():
            self.randomize_mass(link, mass)

        # Friction randomization
        ground_friction = self.randomize_friction(1.0, 0.3)

        # Joint dynamics
        joint_dampings = {
            'left_hip_pitch': 5.0,
            'left_knee_pitch': 3.0,
            'left_ankle_pitch': 2.0,
        }
        for joint, damping in joint_dampings.items():
            self.randomize_joint_dynamics(joint, damping)
```

### Sensor Noise Injection

```python
# sensor_noise.py
import numpy as np

class SensorNoiseModel:
    def __init__(self):
        # IMU noise parameters
        self.gyro_noise_std = 0.01        # rad/s
        self.gyro_bias_std = 0.001        # rad/s
        self.accel_noise_std = 0.1        # m/s²
        self.accel_bias_std = 0.01        # m/s²

        # Camera noise
        self.image_noise_std = 5          # pixel intensity

        # Joint encoder noise
        self.encoder_noise_std = 0.001    # rad

        # Initialize biases (constant during episode)
        self.reset_biases()

    def reset_biases(self):
        """Reset sensor biases at episode start."""
        self.gyro_bias = np.random.normal(0, self.gyro_bias_std, 3)
        self.accel_bias = np.random.normal(0, self.accel_bias_std, 3)

    def add_imu_noise(self, gyro, accel):
        """Add noise to IMU readings."""
        noisy_gyro = gyro + self.gyro_bias + np.random.normal(
            0, self.gyro_noise_std, 3
        )
        noisy_accel = accel + self.accel_bias + np.random.normal(
            0, self.accel_noise_std, 3
        )
        return noisy_gyro, noisy_accel

    def add_encoder_noise(self, positions):
        """Add noise to joint encoders."""
        noise = np.random.normal(0, self.encoder_noise_std, len(positions))
        return positions + noise

    def add_image_noise(self, image):
        """Add Gaussian noise to camera image."""
        noise = np.random.normal(0, self.image_noise_std, image.shape)
        return np.clip(image + noise, 0, 255).astype(np.uint8)
```

### Visual Randomization

```python
# visual_randomization.py
import omni.replicator.core as rep

def randomize_visuals():
    """Randomize visual appearance for robust perception."""

    # Lighting variation
    with rep.trigger.on_frame():
        # Randomize light intensity and color
        lights = rep.create.light(
            light_type="Dome",
            intensity=rep.distribution.uniform(500, 2000),
            color=rep.distribution.uniform((0.9, 0.9, 0.9), (1.1, 1.1, 1.1))
        )

        # Randomize object textures
        objects = rep.get.prims(semantics=[("class", "obstacle")])
        with objects:
            rep.randomizer.materials(
                materials=[
                    "omniverse://localhost/NVIDIA/Materials/Base/Wood/*",
                    "omniverse://localhost/NVIDIA/Materials/Base/Metal/*",
                    "omniverse://localhost/NVIDIA/Materials/Base/Plastic/*",
                ]
            )

        # Randomize floor texture
        floor = rep.get.prims(path_pattern="/World/Floor")
        with floor:
            rep.randomizer.materials(
                materials=[
                    "omniverse://localhost/NVIDIA/Materials/Base/Concrete/*",
                    "omniverse://localhost/NVIDIA/Materials/Base/Wood/Floor/*",
                ]
            )
```

---

## System Identification

### Identifying Real Robot Parameters

```python
# system_identification.py
import numpy as np
from scipy.optimize import minimize

class SystemIdentifier:
    def __init__(self, real_data, sim_model):
        self.real_data = real_data
        self.sim_model = sim_model

    def compute_error(self, params):
        """Compute error between sim and real trajectories."""
        # Update simulation parameters
        self.sim_model.set_parameters(params)

        # Run simulation with same inputs as real experiment
        sim_trajectory = self.sim_model.simulate(
            self.real_data['commands']
        )

        # Compute trajectory error
        error = np.mean(
            (sim_trajectory - self.real_data['states']) ** 2
        )
        return error

    def identify(self, initial_params, bounds):
        """Run optimization to find best parameters."""
        result = minimize(
            self.compute_error,
            initial_params,
            method='L-BFGS-B',
            bounds=bounds
        )
        return result.x

# Usage
identifier = SystemIdentifier(real_robot_data, simulation_model)
identified_params = identifier.identify(
    initial_params=[1.0, 5.0, 0.1],  # mass, damping, friction
    bounds=[(0.5, 2.0), (1.0, 10.0), (0.05, 0.3)]
)
```

### Motor Model Calibration

```python
# motor_calibration.py
class MotorModel:
    def __init__(self):
        self.kt = 0.1           # Torque constant (Nm/A)
        self.kv = 0.1           # Back-EMF constant (V/rad/s)
        self.R = 1.0            # Resistance (Ohm)
        self.L = 0.001          # Inductance (H)
        self.J = 0.01           # Rotor inertia (kg·m²)
        self.b = 0.001          # Viscous friction (Nm·s/rad)

    def step(self, voltage, velocity, dt):
        """Simulate one timestep of motor dynamics."""
        # Back-EMF
        back_emf = self.kv * velocity

        # Current (simplified, ignoring inductance for small dt)
        current = (voltage - back_emf) / self.R

        # Torque
        torque = self.kt * current - self.b * velocity

        return torque, current

    def calibrate_from_data(self, voltage_data, velocity_data, torque_data):
        """Calibrate motor parameters from experimental data."""
        from scipy.optimize import curve_fit

        def motor_torque(v_vel, kt, kv, R, b):
            voltage, velocity = v_vel
            current = (voltage - kv * velocity) / R
            return kt * current - b * velocity

        popt, _ = curve_fit(
            lambda x, kt, kv, R, b: motor_torque((x[:, 0], x[:, 1]), kt, kv, R, b),
            np.column_stack([voltage_data, velocity_data]),
            torque_data,
            p0=[self.kt, self.kv, self.R, self.b]
        )

        self.kt, self.kv, self.R, self.b = popt
```

---

## Reinforcement Learning Transfer

### Domain Randomization in RL Training

```python
# rl_domain_randomization.py
import gymnasium as gym
import numpy as np

class RandomizedHumanoidEnv(gym.Env):
    def __init__(self, base_env):
        self.base_env = base_env
        self.action_space = base_env.action_space
        self.observation_space = base_env.observation_space

        # Randomization ranges
        self.mass_range = (0.8, 1.2)
        self.friction_range = (0.5, 1.5)
        self.latency_range = (0, 20)  # ms

    def reset(self, **kwargs):
        # Randomize physics
        mass_scale = np.random.uniform(*self.mass_range)
        friction = np.random.uniform(*self.friction_range)
        self.latency = np.random.randint(*self.latency_range)

        self.base_env.set_mass_scale(mass_scale)
        self.base_env.set_friction(friction)

        # Action delay buffer
        self.action_buffer = []

        return self.base_env.reset(**kwargs)

    def step(self, action):
        # Simulate action latency
        self.action_buffer.append(action)
        if len(self.action_buffer) > self.latency:
            delayed_action = self.action_buffer.pop(0)
        else:
            delayed_action = np.zeros_like(action)

        # Add action noise
        noisy_action = delayed_action + np.random.normal(0, 0.01, action.shape)

        obs, reward, terminated, truncated, info = self.base_env.step(noisy_action)

        # Add observation noise
        noisy_obs = obs + np.random.normal(0, 0.01, obs.shape)

        return noisy_obs, reward, terminated, truncated, info
```

### Adaptive Domain Randomization

```python
# adaptive_domain_randomization.py
class AdaptiveDomainRandomization:
    def __init__(self, param_ranges, initial_difficulty=0.1):
        self.param_ranges = param_ranges
        self.difficulty = initial_difficulty
        self.success_history = []

    def get_randomized_params(self):
        """Get parameters based on current difficulty."""
        params = {}
        for name, (low, high) in self.param_ranges.items():
            # Interpolate based on difficulty
            center = (low + high) / 2
            range_size = (high - low) * self.difficulty
            params[name] = np.random.uniform(
                center - range_size / 2,
                center + range_size / 2
            )
        return params

    def update_difficulty(self, success_rate, target_rate=0.7):
        """Adjust difficulty based on success rate."""
        self.success_history.append(success_rate)

        if len(self.success_history) >= 10:
            avg_success = np.mean(self.success_history[-10:])

            if avg_success > target_rate + 0.1:
                self.difficulty = min(1.0, self.difficulty * 1.1)
            elif avg_success < target_rate - 0.1:
                self.difficulty = max(0.1, self.difficulty * 0.9)
```

---

## Transfer Validation

### Real-World Testing Protocol

```python
# transfer_validation.py
class TransferValidator:
    def __init__(self, policy, real_robot):
        self.policy = policy
        self.robot = real_robot

    def run_validation_suite(self):
        """Run comprehensive transfer validation."""
        results = {}

        # Test 1: Static balance
        results['static_balance'] = self.test_static_balance()

        # Test 2: Walking on flat ground
        results['flat_walking'] = self.test_flat_walking()

        # Test 3: Disturbance rejection
        results['disturbance'] = self.test_disturbance_rejection()

        # Test 4: Uneven terrain
        results['uneven_terrain'] = self.test_uneven_terrain()

        return results

    def test_static_balance(self, duration=30.0):
        """Test standing balance for specified duration."""
        self.robot.stand()
        start_time = time.time()
        stable = True

        while time.time() - start_time < duration:
            obs = self.robot.get_observation()
            action = self.policy.get_action(obs)
            self.robot.apply_action(action)

            if self.robot.has_fallen():
                stable = False
                break

        return {
            'passed': stable,
            'duration': time.time() - start_time
        }

    def test_disturbance_rejection(self, force_magnitudes=[10, 20, 30]):
        """Test recovery from external pushes."""
        results = []
        for force in force_magnitudes:
            self.robot.stand()
            time.sleep(2.0)

            # Apply disturbance
            self.robot.apply_external_force(force, direction='forward')

            # Check recovery
            recovered = self.robot.wait_for_stable(timeout=5.0)
            results.append({
                'force': force,
                'recovered': recovered
            })

        return results
```

---

## Best Practices

### 1. Start Simple

```python
# Progressive transfer strategy
transfer_stages = [
    {'physics_randomization': 0.05, 'sensor_noise': 0.01},
    {'physics_randomization': 0.10, 'sensor_noise': 0.02},
    {'physics_randomization': 0.20, 'sensor_noise': 0.05},
    {'physics_randomization': 0.30, 'sensor_noise': 0.10},
]

for stage in transfer_stages:
    train_with_settings(stage)
    if validate_on_real_robot():
        continue
    else:
        break  # More training needed
```

### 2. Gradual Deployment

1. Test in controlled lab environment
2. Add safety constraints (fall detection, joint limits)
3. Increase task complexity gradually
4. Monitor and log all deployments

### 3. Continuous Improvement

```python
# Collect real-world data for sim improvement
def collect_and_improve():
    # Deploy policy
    real_trajectory = deploy_on_robot()

    # Compare with simulation
    sim_trajectory = simulate_same_commands()

    # Identify discrepancies
    discrepancy = analyze_difference(real_trajectory, sim_trajectory)

    # Update simulation parameters
    update_simulation(discrepancy)

    # Retrain if necessary
    if discrepancy > threshold:
        retrain_policy()
```

---

## Summary

This chapter covered sim-to-real transfer:

*   **Domain randomization**: Physics, sensors, visuals
*   **System identification**: Calibrating simulation to reality
*   **RL transfer**: Training robust policies
*   **Validation**: Testing transferred policies
*   **Best practices**: Progressive deployment strategies

Effective sim-to-real transfer is essential for deploying humanoid robots safely and reliably. In the next module, we will explore Vision-Language-Action systems for natural interaction.
