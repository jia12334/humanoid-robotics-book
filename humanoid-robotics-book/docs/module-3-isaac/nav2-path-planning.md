# Chapter 17: Nav2 Path Planning

Nav2 (Navigation2) is the ROS 2 navigation stack providing autonomous navigation capabilities. This chapter covers configuring Nav2 for humanoid robots, including path planning, behavior trees, and obstacle avoidance.

---

## Nav2 Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    Behavior Tree (BT)                       │
│  ┌─────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │Navigate │  │  Spin    │  │  Wait    │  │   Backup    │  │
│  │ To Pose │  │          │  │          │  │             │  │
│  └────┬────┘  └────┬─────┘  └────┬─────┘  └──────┬──────┘  │
└───────┼────────────┼─────────────┼───────────────┼──────────┘
        │            │             │               │
        ▼            ▼             ▼               ▼
┌─────────────────────────────────────────────────────────────┐
│                      Nav2 Servers                           │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │   Planner   │  │  Controller  │  │    Recovery       │  │
│  │   Server    │  │   Server     │  │    Server         │  │
│  └──────┬──────┘  └──────┬───────┘  └─────────┬─────────┘  │
└─────────┼────────────────┼────────────────────┼─────────────┘
          │                │                    │
          ▼                ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│                      Costmaps                               │
│         Global Costmap         │      Local Costmap         │
└─────────────────────────────────────────────────────────────┘
```

---

## Installation and Setup

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install for simulation
sudo apt install ros-humble-nav2-simple-commander
```

---

## Configuration

### Nav2 Parameters

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

---

## Humanoid-Specific Configuration

### Footprint Configuration

```yaml
# For bipedal robots with rectangular footprint
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.15, 0.10], [0.15, -0.10], [-0.10, -0.10], [-0.10, 0.10]]"

# For circular approximation
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.25
```

### Velocity Constraints

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # Humanoid walking speeds
      min_vel_x: 0.0
      max_vel_x: 0.3        # ~1 km/h walking
      max_vel_y: 0.1        # Limited lateral movement
      max_vel_theta: 0.5    # Slow rotation

      # Acceleration limits for stability
      acc_lim_x: 0.5
      acc_lim_y: 0.2
      acc_lim_theta: 0.5
```

---

## Path Planners

### NavFn Planner (Default)

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false
```

### Smac Hybrid-A* Planner

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      motion_model_for_search: "DUBIN"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.4
      reverse_penalty: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.0
```

---

## Controllers

### DWB Local Planner

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      trajectory_generator_name: "dwb_plugins::StandardTrajectoryGenerator"
      goal_checker_name: "dwb_plugins::SimpleGoalChecker"

      # Trajectory scoring
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
```

### Regulated Pure Pursuit

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 0.5
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 0.5
```

---

## Behavior Trees

### Custom Navigation BT

```xml
<!-- navigate_humanoid.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <!-- Compute path -->
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <!-- Follow path -->
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <!-- Recovery behaviors -->
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
        <BackUp backup_dist="0.3" backup_speed="0.1"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Humanoid-Safe Recovery

```xml
<!-- Safe recovery for bipedal robots -->
<SequenceStar name="HumanoidRecovery">
  <!-- First try clearing costmaps -->
  <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>

  <!-- Slow spin (safer for balance) -->
  <Spin spin_dist="0.785" time_allowance="10"/>

  <!-- Wait for obstacles to clear -->
  <Wait wait_duration="3"/>

  <!-- Small careful backup -->
  <BackUp backup_dist="0.15" backup_speed="0.05"/>
</SequenceStar>
```

---

## Using Nav2 Simple Commander

```python
#!/usr/bin/env python3
# nav2_commander.py

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()

    # Define goal
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0

    # Navigate
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal reached!')
    elif result == TaskResult.CANCELED:
        print('Navigation canceled')
    elif result == TaskResult.FAILED:
        print('Navigation failed')

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Waypoint Following

```python
# Multi-waypoint navigation
def navigate_through_waypoints(navigator, waypoints):
    """Navigate through a list of waypoints."""
    goal_poses = []

    for wp in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp['x']
        goal_pose.pose.position.y = wp['y']
        goal_pose.pose.orientation.w = 1.0
        goal_poses.append(goal_pose)

    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            current_wp = feedback.current_waypoint
            print(f'Executing waypoint {current_wp + 1}/{len(waypoints)}')

    return navigator.getResult()

# Usage
waypoints = [
    {'x': 1.0, 'y': 0.0},
    {'x': 2.0, 'y': 1.0},
    {'x': 1.0, 'y': 2.0},
    {'x': 0.0, 'y': 1.0},
]
navigate_through_waypoints(navigator, waypoints)
```

---

## Launch File

```python
# launch/nav2_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    humanoid_nav_dir = get_package_share_directory('humanoid_navigation')

    params_file = os.path.join(humanoid_nav_dir, 'config', 'nav2_params.yaml')
    bt_file = os.path.join(humanoid_nav_dir, 'behavior_trees', 'navigate_humanoid.xml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'default_bt_xml_filename': bt_file,
                'use_sim_time': 'true',
            }.items()
        ),
    ])
```

---

## Summary

This chapter covered Nav2 path planning:

*   **Architecture**: BT navigator, planners, controllers, costmaps
*   **Configuration**: Parameters for humanoid robots
*   **Path planners**: NavFn, Smac Hybrid-A*
*   **Controllers**: DWB, Regulated Pure Pursuit
*   **Behavior trees**: Custom navigation behaviors
*   **Simple Commander**: Python API for navigation

Nav2 provides robust autonomous navigation for humanoid robots. In the next chapter, we will explore sim-to-real transfer techniques.
