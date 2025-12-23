# Chapter Contract: Ch4 - Path Planning and Navigation with Nav2

**File**: `frontend/docs/module-03-nvidia-isaac/04-nav2-navigation.md`
**Word Target**: 1500-2500 words
**Priority**: P4

## Frontmatter

```yaml
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
```

## Required Sections

### 1. Introduction (~200 words)
- Connect to Chapter 3: Perception provides location, now we need to navigate
- Nav2 is the standard ROS 2 navigation stack
- Challenge: Originally designed for wheeled robots, humanoids need adaptation

### 2. Nav2 Architecture Overview (~500 words)
- **FR-009**: MUST explain global planner, local planner, costmap
- **Behavior Tree Navigator**:
  - Orchestrates navigation actions
  - XML-defined behavior trees
  - Recovery behaviors (spin, backup, wait)
- **Global Planner**:
  - Computes path from start to goal
  - Operates on static + known obstacles
  - Algorithms: NavFn (Dijkstra/A*), Smac (state lattice)
- **Local Planner (Controller)**:
  - Follows path segment in real-time
  - Reacts to dynamic obstacles
  - Algorithms: DWB, TEB, MPPI, Regulated Pure Pursuit
- **Costmap Manager**:
  - Layered representation of environment
  - Static layer (known map)
  - Obstacle layer (sensor detections)
  - Inflation layer (safety margins)

### 3. Perception-to-Planning Integration (~400 words)
- **FR-011**: MUST show how perception/localization outputs integrate
- From Isaac ROS:
  - Odometry → robot pose in odom frame
  - Localization → pose in map frame
  - Depth/Point Cloud → obstacle detection
- Into Nav2:
  - TF transforms (map → odom → base_link)
  - Costmap updates from sensor data
  - Global planner uses localized pose for planning

### 4. Humanoid Navigation Adaptations (~500 words)
- **FR-010**: MUST describe adaptations for bipedal humanoids

**Challenge 1: Discrete Footsteps**
- Wheeled robots: continuous velocity execution
- Humanoids: discrete foot placements
- Adaptation: Footstep planning layer between Nav2 and gait controller

**Challenge 2: Balance Constraints**
- Wheeled robots: stable at rest and motion
- Humanoids: active balance required
- Adaptation: Velocity profiles must respect stability margins

**Challenge 3: Dynamic Footprint**
- Wheeled robots: fixed robot footprint
- Humanoids: footprint changes with stance
- Adaptation: Stance-aware costmap footprint

**Challenge 4: Fall-Aware Recovery**
- Wheeled robots: rotate in place, backup
- Humanoids: cannot simply spin without falling
- Adaptation: Stability-preserving recovery behaviors

### 5. Connecting the Full Pipeline (~300 words)
- End-to-end walkthrough:
  1. Isaac Sim: Train perception models
  2. Isaac ROS: Real-time perception + localization
  3. Nav2: Path planning + execution
  4. ROS 2 Control: Joint commands (Module 1)
  5. Robot Hardware: Physical motion
- The complete "AI brain" in action

### 6. Key Takeaways
- Nav2 = Behavior Trees + Planners + Costmaps
- Global planner: start-to-goal path, Local planner: real-time execution
- Costmap layers: static, obstacle, inflation
- Humanoids need footstep planning, balance-aware control, stability-preserving recovery
- Full pipeline: sensors → perception → localization → planning → control

### 7. Next Steps
- Module 3 complete—reader understands Isaac as AI brain
- Preview Module 4: Vision-Language-Action for higher-level intelligence

## Requirements Coverage

| Requirement | Section | How Addressed |
|-------------|---------|---------------|
| FR-009 | Section 2 | Global planner, local planner, costmap explained |
| FR-010 | Section 4 | Four humanoid-specific adaptations described |
| FR-011 | Section 3 | Perception/localization → Nav2 integration |
| FR-012 | All | Word count 1500-2500 |
| FR-013 | All | Technical, non-marketing tone |
| FR-014 | Snippet | Conceptual behavior tree XML |

## Acceptance Scenarios

1. **Given** Nav2 architecture explanation, **When** reader completes chapter, **Then** they can explain roles of global planner, local planner, costmap
2. **Given** humanoid-specific challenges, **When** asked about bipedal navigation, **Then** reader can identify differences from wheeled navigation
3. **Given** integration context, **When** shown full pipeline, **Then** reader can explain how perception feeds Nav2

## Conceptual Code Snippet

```xml
<!-- Conceptual Nav2 Behavior Tree for Navigation -->
<!-- Shows structure of navigation logic -->
<!-- NOT executable without full Nav2 setup -->

<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <!-- Main navigation pipeline -->
      <PipelineSequence name="NavigateWithReplanning">
        <!-- Compute global path -->
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        <!-- Follow path with local planner -->
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>

      <!-- Recovery behaviors (humanoid-adapted) -->
      <ReactiveFallback name="RecoveryFallback">
        <!-- Wait for obstacle to clear -->
        <Wait wait_duration="5.0"/>
        <!-- Careful backup (balance-aware) -->
        <BackUp backup_dist="0.2" backup_speed="0.1"/>
        <!-- Request human assistance if stuck -->
        <AlwaysFailure/>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Diagram Description

:::info Diagram: Nav2 Architecture

**Type**: architecture

**Description**: Shows Nav2 components and their interactions.

**Key Elements**:
- Behavior Tree Navigator (top, orchestrates)
- Global Planner (NavFn, Smac) → produces Path
- Local Planner/Controller (DWB, MPPI) → produces cmd_vel
- Costmap Manager → Static, Obstacle, Inflation layers
- Inputs: Localization (from Isaac ROS), Sensor data
- Output: Velocity commands → ROS 2 Control
- Arrows showing data flow between components

:::

:::info Diagram: Wheeled vs Humanoid Navigation

**Type**: comparison

**Description**: Side-by-side comparison of navigation approaches.

**Key Elements**:
- Left: Wheeled Robot
  - Continuous path
  - Direct velocity execution
  - Fixed footprint
  - Rotate-in-place recovery
- Right: Humanoid Robot
  - Discrete footsteps
  - Balance-constrained motion
  - Dynamic footprint
  - Stability-preserving recovery
- Center: Shared components (global planner, costmap)

:::

## Chunk Map

- `chunk:nav2-architecture` - BT, planners, costmaps overview
- `chunk:global-vs-local-planning` - Path computation vs trajectory following
- `chunk:costmap-layers` - Static, obstacle, inflation purposes
- `chunk:humanoid-navigation` - Bipedal-specific challenges
- `chunk:footstep-planning-concept` - Discrete step placement layer
- `chunk:full-pipeline-integration` - End-to-end data flow
