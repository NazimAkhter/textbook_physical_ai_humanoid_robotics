---
id: nav2-navigation
title: "Path Planning and Navigation with Nav2"
sidebar_label: "Nav2 Navigation"
sidebar_position: 4
description: "Learn Nav2 architecture for autonomous navigation and how to adapt it for bipedal humanoid robots."
keywords: ["nav2", "ros2 navigation", "path planning", "costmap", "behavior tree", "humanoid navigation", "footstep planning"]
learning_objectives:
  - "Explain Nav2 architecture including global planner, local planner, and costmap"
  - "Describe at least two adaptations required for bipedal humanoid navigation"
  - "Trace how perception and localization outputs integrate with Nav2 planning"
---

# Path Planning and Navigation with Nav2

<!-- chunk:nav2-introduction -->

## Introduction

Chapter 3 covered Isaac ROS perception—how GPU-accelerated processing transforms camera images into localization and obstacle information. Navigation is where this perception becomes action: computing paths through the environment and executing motion to reach goals.

Nav2 is the standard ROS 2 navigation stack, providing a complete framework for autonomous robot navigation. While Nav2 was originally designed for wheeled mobile robots, its modular architecture allows adaptation for different robot types, including bipedal humanoids.

This chapter examines Nav2's architecture and the specific challenges humanoid robots present for navigation. Understanding these challenges clarifies why humanoid navigation remains an active research area and what adaptations are necessary beyond standard Nav2 deployment.

<!-- /chunk -->

<!-- chunk:nav2-architecture -->

## Nav2 Architecture Overview

Nav2 organizes navigation into modular components with well-defined interfaces. This architecture enables customization—replacing individual components while maintaining system integration.

### Behavior Tree Navigator

The Behavior Tree (BT) Navigator orchestrates navigation actions through configurable behavior trees. Unlike finite state machines that define explicit transitions between states, behavior trees compose actions and conditions into hierarchical structures:

```xml
<!-- Conceptual Nav2 Behavior Tree for Navigation -->
<!-- Shows structure of navigation logic -->
<!-- NOT executable without full Nav2 configuration -->

<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <!-- Main navigation pipeline -->
      <PipelineSequence name="NavigateWithReplanning">
        <!-- Compute global path from current pose to goal -->
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        <!-- Follow computed path with local controller -->
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>

      <!-- Recovery behaviors when navigation fails -->
      <ReactiveFallback name="RecoveryFallback">
        <Wait wait_duration="5.0"/>
        <BackUp backup_dist="0.3" backup_speed="0.1"/>
        <Spin spin_dist="1.57"/>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

Key behavior tree concepts:

- **Sequences** execute children in order, succeeding only if all children succeed
- **Fallbacks** try children until one succeeds
- **Recovery nodes** attempt main behavior, falling back to recovery actions on failure
- **Conditions** check state without causing actions

This structure provides predictable yet flexible navigation behavior. When path following fails (obstacle blocks route), the recovery fallback attempts progressively more aggressive recovery actions before giving up.

### Global Planner

The global planner computes complete paths from the robot's current position to the navigation goal. Operating on the static costmap layer plus known obstacles, global planning answers: "What sequence of positions leads from here to the goal?"

Common global planner algorithms:

| Algorithm | Approach | Characteristics |
|-----------|----------|-----------------|
| **NavFn** | Dijkstra/A* on grid | Fast, complete, grid-resolution paths |
| **Smac 2D** | State lattice on grid | Respects non-holonomic constraints |
| **Smac Hybrid-A*** | Continuous state search | Smooth paths for car-like robots |
| **Theta*** | Any-angle pathfinding | Shorter paths than grid-constrained |

For humanoid robots, global planners produce waypoint sequences that local planning and locomotion controllers must achieve. The global path doesn't specify *how* to reach each waypoint—that's the local planner's responsibility.

### Local Planner (Controller)

The local planner generates real-time velocity commands that follow the global path while avoiding dynamic obstacles. Operating at 10-20 Hz, local planning must react quickly to environmental changes.

Controller options include:

**DWB (Dynamic Window Approach)**: Samples candidate velocities, simulates their trajectories, and scores based on path following, obstacle distance, and goal progress. Well-understood and widely used.

**TEB (Timed Elastic Band)**: Optimizes trajectories considering time, finding paths that satisfy kinematic constraints while minimizing travel time.

**MPPI (Model Predictive Path Integral)**: Samples many trajectories using learned or physics-based models, weighing outcomes by cost. Handles complex dynamics well but requires more computation.

**Regulated Pure Pursuit**: Simple geometric path following with speed regulation near obstacles. Fast and predictable but limited handling of complex scenarios.

### Costmap Manager

The costmap represents navigable space as a grid where each cell contains a cost value:

- **Free space (0)**: Safe to traverse
- **Lethal (254)**: Occupied by obstacle
- **Inscribed (253)**: Would cause collision given robot footprint
- **Inflated (1-252)**: Gradient away from obstacles, penalizing proximity

Costmaps are built from layers that combine to produce the final cost grid:

**Static layer**: Loaded from a pre-built map (from SLAM or surveying)

**Obstacle layer**: Updated from sensor observations—depth cameras, LiDAR, point clouds from Isaac ROS

**Inflation layer**: Expands obstacles by the robot's inscribed radius plus a safety margin

**Voxel layer**: 3D obstacle representation for robots that must consider height (relevant for humanoid head clearance)

<!-- /chunk -->

<!-- chunk:perception-planning-integration -->

## Perception-to-Planning Integration

Isaac ROS perception outputs connect to Nav2 through standard ROS 2 interfaces. Understanding this integration clarifies data flow from sensors to motion.

### Localization Integration

Nav2 requires robot pose in the map frame to plan paths and track progress. Isaac ROS Visual SLAM provides this through:

**TF transforms**: The SLAM system publishes the `map → odom` transform, connecting the odometry frame to the global map. Combined with the robot's `odom → base_link` transform (from odometry), Nav2 can compute the robot's map-frame pose.

**Odometry topic**: Continuous velocity and pose estimates on `/odom` feed the local planner for responsive trajectory following.

**Localization status**: When SLAM confidence is low (tracking loss, rapid motion), navigation can pause or slow down rather than continuing with unreliable pose estimates.

### Obstacle Integration

Depth and detection outputs update the costmap:

**Point cloud insertion**: The `obstacle_layer` plugin subscribes to point cloud topics (from Isaac ROS depth processing). Each observation marks cells as occupied and clears cells along sensor rays as free.

**Sensor configuration**: Marking and clearing parameters control how aggressively observations update the costmap. For humanoid robots with dynamic body motion, careful tuning prevents self-observation from corrupting the map.

**Temporal filtering**: Brief detections (noise) shouldn't immediately block navigation; persistent obstacles should quickly appear. Layer parameters balance responsiveness with stability.

### The Navigation Loop

With perception and costmap configured, navigation proceeds as a continuous loop:

1. **Goal received**: Navigation action accepts target pose
2. **Global path computed**: Planner finds route through costmap
3. **Local control begins**: Controller generates velocity commands
4. **Costmap updates**: Perception continuously refines obstacle knowledge
5. **Replanning triggered**: Path adjusts if obstacles invalidate current route
6. **Goal reached or failed**: Navigation action returns result

This loop runs until the goal is achieved, cancelled, or deemed unreachable.

<!-- /chunk -->

<!-- chunk:humanoid-navigation -->

## Humanoid Navigation Adaptations

Nav2's default configuration assumes differential-drive or holonomic wheeled robots. Bipedal humanoids present fundamentally different challenges that require architectural adaptation.

### Challenge 1: Discrete Footsteps vs Continuous Motion

Wheeled robots translate velocity commands directly into wheel rotation. Humanoids must plan and execute discrete foot placements:

| Wheeled Robot | Humanoid Robot |
|---------------|----------------|
| Continuous velocity execution | Discrete step sequences |
| Smooth trajectory following | Step-by-step progression |
| Instantaneous direction change | Multi-step turning sequences |
| Velocity = direct motor command | Velocity = gait pattern selection |

**Adaptation**: A footstep planning layer sits between Nav2's velocity output and the locomotion controller. This layer converts continuous paths into feasible step sequences respecting step length, width, and terrain constraints.

### Challenge 2: Balance Constraints

Wheeled robots are statically stable—they won't tip over when stopped. Humanoids require active balance:

- Standing still requires continuous balance control
- Walking shifts the center of mass outside the support polygon
- External disturbances (pushes, uneven terrain) require reactive compensation
- Velocity commands must be achievable while maintaining balance

**Adaptation**: The local planner must respect dynamic balance constraints. Velocity profiles that would cause instability (sudden accelerations, abrupt stops) must be smoothed or rejected. Some approaches integrate balance predictions into path cost functions.

### Challenge 3: Variable Footprint

A wheeled robot's collision footprint is fixed—the robot body never changes shape during navigation. Humanoids have a dynamic footprint:

- Stance phase: Both feet on ground, wide base
- Swing phase: One foot lifted, narrow support
- Arm motion: Reaching or carrying changes collision envelope
- Turning: Footprint rotates relative to body frame

**Adaptation**: Costmap footprint models must update based on current stance phase or use worst-case bounds. Some implementations maintain separate footprints for planning (conservative) and execution (actual).

### Challenge 4: Fall-Aware Recovery

Standard Nav2 recovery behaviors assume wheeled robot capabilities:

- **Spin in place**: Humanoids cannot spin without stepping
- **Back up**: Walking backward requires different gait patterns
- **Wait**: Safe for wheeled robots; humanoids may struggle to maintain balance indefinitely

**Adaptation**: Recovery behaviors must be humanoid-aware. Stepping turns replace spinning. Backward walking uses appropriate gait. Extended waiting may require posture adjustments. Fall prediction should trigger protective actions before instability.

<!-- /chunk -->

<!-- chunk:full-pipeline -->

## Connecting the Full Pipeline

With all Module 3 components in place, the complete perception-to-action pipeline operates continuously:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Complete Navigation Pipeline                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Training Loop (Offline)                                           │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐            │
│  │ Isaac Sim   │───▶│  Synthetic  │───▶│   Trained   │            │
│  │ (Chapter 2) │    │    Data     │    │   Models    │            │
│  └─────────────┘    └─────────────┘    └─────────────┘            │
│                                              │                      │
│  ════════════════════════════════════════════╪══════════════════   │
│                                              │ Deploy               │
│  Execution Loop (Real-time)                  ▼                      │
│  ┌─────────────┐    ┌─────────────────────────────────┐           │
│  │   Sensors   │───▶│      Isaac ROS (Chapter 3)      │           │
│  │             │    │  • Visual SLAM → Localization   │           │
│  └─────────────┘    │  • Depth → Obstacles            │           │
│                     │  • Detection → Scene            │           │
│                     └────────────┬────────────────────┘           │
│                                  │                                  │
│                                  ▼                                  │
│                     ┌─────────────────────────────────┐           │
│                     │       Nav2 (Chapter 4)          │           │
│                     │  • Costmap ← Obstacles          │           │
│                     │  • Global Planner → Path        │           │
│                     │  • Local Planner → Velocity     │           │
│                     └────────────┬────────────────────┘           │
│                                  │                                  │
│                                  ▼                                  │
│                     ┌─────────────────────────────────┐           │
│                     │   ROS 2 Control (Module 1)      │           │
│                     │  • Gait Controller              │           │
│                     │  • Balance Control              │           │
│                     │  • Joint Commands               │           │
│                     └────────────┬────────────────────┘           │
│                                  │                                  │
│                                  ▼                                  │
│                     ┌─────────────┐                                │
│                     │  Actuators  │                                │
│                     └─────────────┘                                │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

This architecture demonstrates how Isaac components integrate:

1. **Isaac Sim** trains perception models on synthetic data (offline)
2. **Isaac ROS** deploys those models for real-time perception (online)
3. **Nav2** uses perception outputs for autonomous navigation
4. **ROS 2 Control** executes motion through humanoid-specific controllers

The result is an autonomous humanoid robot capable of perceiving its environment, planning paths through complex spaces, and executing bipedal locomotion to achieve navigation goals.

<!-- /chunk -->

## Key Takeaways

- **Nav2 provides modular navigation** through behavior trees, global planners, local planners, and costmap management
- **Global planners compute complete paths** while local planners generate real-time velocity commands
- **Costmaps represent navigable space** through layers combining static maps, sensor observations, and safety inflation
- **Humanoid navigation requires adaptation** for discrete footsteps, balance constraints, variable footprint, and fall-aware recovery
- **The complete pipeline connects** Isaac Sim training, Isaac ROS perception, Nav2 planning, and ROS 2 control into an autonomous navigation system

## Next Steps

Module 3 is complete. You now understand how NVIDIA Isaac provides the AI brain for humanoid robots—from photorealistic simulation and synthetic data generation, through GPU-accelerated perception and localization, to autonomous navigation with Nav2.

Module 4 explores Vision-Language-Action (VLA) systems, where robots learn to perform manipulation tasks from vision and language inputs. These systems represent the next frontier in robot intelligence, enabling humanoids to understand and execute complex, language-described tasks.
