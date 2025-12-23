---
id: end-to-end-integration
title: "End-to-End Autonomous Integration"
sidebar_label: "End-to-End Integration"
sidebar_position: 4
description: "Integrate all VLA components with ROS 2, digital twins, Isaac ROS, and Nav2 to achieve complete autonomous humanoid behavior. Trace natural language commands through the entire system from speech to physical execution."
keywords: ["end-to-end integration", "autonomous systems", "system integration", "complete VLA pipeline", "capstone", "humanoid robotics", "full stack"]
learning_objectives:
  - "Trace a natural language command through the entire system from speech input to physical execution"
  - "Explain how ROS 2, digital twins, Isaac perception/navigation, and VLA components work together"
  - "Describe the complete autonomous humanoid architecture"
---

# End-to-End Autonomous Integration

<!-- chunk:introduction -->

## Introduction

We've built the complete VLA system layer by layer: **Chapter 1** established VLA foundations—the convergence of vision, language, and action. **Chapter 2** detailed voice-to-action pipelines for translating speech to robot commands. **Chapter 3** explored LLM-based cognitive planning for multi-step task decomposition. Now, **Chapter 4 integrates all components** into a complete autonomous humanoid system capable of understanding natural language, reasoning about tasks, and executing physical actions.

This capstone chapter demonstrates how **all four modules** work together:
- **Module 1 (ROS 2)**: Communication infrastructure and control
- **Module 2 (Digital Twins)**: Simulation for testing and validation
- **Module 3 (Isaac ROS + Nav2)**: Perception and navigation
- **Module 4 (VLA)**: Cognitive layer for language understanding and planning

We'll trace the complete data flow from speech input to physical execution, examine Isaac ROS and Nav2 integration, present the full autonomous humanoid architecture, and walk through a detailed capstone example: **"Go to the kitchen and bring me a cup."**

<!-- /chunk -->

<!-- chunk:complete-pipeline -->

## Complete VLA Pipeline

Let's trace the complete data flow for the command **"Go to the kitchen and bring me a cup"** through all seven stages of the VLA system.

### Stage 1: Speech Input

The user speaks the command, and the robot's **microphone array** (hardware layer) captures the audio stream. Multi-microphone beamforming improves signal quality by filtering background noise and focusing on the speaker's direction.

### Stage 2: Speech Recognition (~500ms)

The audio stream feeds into the **Speech Recognition module** (Chapter 2), which converts speech to text:
- **Input**: Audio waveform
- **Processing**: Whisper, Google Cloud STT, or Azure Speech Services
- **Output**: `"Go to the kitchen and bring me a cup"`
- **Latency**: ~500ms on GPU

The speech recognition node publishes text to a ROS 2 topic (`/speech/transcription`), making it available to downstream components.

### Stage 3: Intent Extraction (&lt;100ms)

The **Natural Language Understanding (NLU)** module (Chapter 2) analyzes the transcribed text:
- **Input**: `"Go to the kitchen and bring me a cup"`
- **Processing**: Intent classification detects this is a **complex multi-step task** requiring LLM planning (not a simple command like "stop")
- **Output**: High-level goal passed to Cognitive Planner
- **Latency**: &lt;100ms (lightweight NLU model)

### Stage 4: LLM Cognitive Planning (1-3s)

The **LLM Cognitive Planner** (Chapter 3) decomposes the goal into an executable action sequence:

**Context Inputs**:
- **Environmental State** (from Isaac ROS, Module 3):
  - Current location: `living_room`
  - Visible objects: `[table, chair]` (cup not yet detected)
  - Obstacle map: Nav2 costmap
- **Robot Capabilities** (from ROS 2, Module 1):
  - Available actions: `navigate`, `detect_object`, `grasp`, `place`, `hand_over`
- **Task History**: None (first command in session)

**LLM Output** (using Chain-of-Thought prompting):
1. `navigate(kitchen)` — Move to kitchen location
2. `detect_object(cup)` — Use Isaac ROS to locate cup
3. `grasp(cup)` — Execute manipulation to pick up cup
4. `navigate(user_location)` — Return to user's current position
5. `hand_over(cup)` — Release cup to user

**Latency**: 1-3s for LLM inference (acceptable for high-level planning)

### Stage 5: Action Mapping

The **Action Mapper** (Chapter 2) translates each abstract action to a **ROS 2 action goal**:
- `navigate(kitchen)` → `NavigateToPose(kitchen_pose)` (Nav2, Module 3)
- `detect_object(cup)` → `DetectObjects(class_id="cup")` (Isaac ROS, Module 3)
- `grasp(cup)` → `Grasp(target_pose)` (manipulation controller, Module 1)

The Action Mapper validates parameters (e.g., kitchen location exists in semantic map) and sends goals to the appropriate ROS 2 action servers.

### Stage 6: Execution

ROS 2 action servers (Modules 1 and 3) execute each goal:
- **Nav2**: Plans collision-free path using costmap (Isaac ROS depth images), sends velocity commands via ROS 2 Control
- **Isaac ROS**: Runs object detection model on camera images, publishes detected object poses
- **Manipulation Controller**: Computes inverse kinematics, executes joint trajectories to grasp cup

### Stage 7: Perception Feedback & Replanning

**Isaac ROS** continuously updates environmental state:
- **During navigation**: Visual SLAM provides localization, costmap updates reflect new obstacles
- **During detection**: Object detection confirms cup presence or triggers replanning if not found
- **During grasping**: Tactile feedback confirms successful grasp

**Feedback Loop** (ReAct pattern from Chapter 3):
- **Success**: `navigate(kitchen) → SUCCESS` → Continue to `detect_object(cup)`
- **Failure**: `detect_object(cup) → FAILURE (cup not visible)` → LLM replans: "Search living room instead" or "Ask user for cup location"

This closed-loop architecture enables **adaptive behavior**—the robot adjusts plans based on real-world feedback rather than executing blindly.

<!-- /chunk -->

<!-- chunk:isaac-integration -->

## Integration with Isaac ROS

The VLA Cognitive Planner depends heavily on **Isaac ROS perception outputs** (Module 3) to understand the environment and plan feasible actions.

### Perception Inputs to VLA

**Visual SLAM (Localization)**:
- Isaac ROS Visual SLAM provides the robot's current pose in the map frame
- Example: `Current location: living_room (x=1.2, y=0.8, θ=0.0)`
- **Use**: LLM context includes robot location, enabling commands like "go back to the kitchen" to reference previous locations

**Object Detection**:
- Isaac ROS runs deep learning models (e.g., DOPE, FoundationPose) to detect objects
- Example: `Cup detected at (x=2.3, y=1.1, z=0.8)` relative to robot base frame
- **Use**: LLM plans `grasp(cup)` action with target pose from detection results

**Semantic Segmentation**:
- Isaac ROS segments scenes into semantic categories (floor, wall, furniture, objects)
- Example: `Kitchen counter detected at (x=4.0, y=2.0)`
- **Use**: LLM understands spatial relationships ("cup on the kitchen counter")

**Depth Estimation**:
- Isaac ROS depth images feed into Nav2 costmap
- Example: Obstacle at `(x=3.0, y=1.5)` blocks direct path to kitchen
- **Use**: Nav2 plans collision-free paths; LLM aware of navigation constraints

### Data Flow Example

```
Isaac ROS Object Detection
    ↓
Publishes to /detected_objects topic (ROS 2)
    ↓
LLM Context Updater subscribes
    ↓
Updates context: "Visible Objects: [cup (x=2.3, y=1.1)]"
    ↓
LLM Cognitive Planner uses context
    ↓
Plans: grasp(cup) with target (2.3, 1.1, 0.8)
    ↓
Action Mapper: Grasp(target_pose=(2.3, 1.1, 0.8))
    ↓
Manipulation Controller executes grasp
```

**Integration Benefit**: Without Isaac ROS perception, the LLM could not ground abstract goals ("bring me a cup") in concrete object poses required for manipulation. Perception closes the gap between language and physical action.

<!-- /chunk -->

<!-- chunk:nav2-integration -->

## Integration with Nav2

The VLA system relies on **Nav2** (Module 3) for collision-free navigation, path planning, and obstacle avoidance.

### Navigation Integration

**LLM Outputs Navigation Goals**:
- LLM plans: `navigate(kitchen)` (abstract location)
- **Semantic Mapping**: A location database maps "kitchen" → `(x=5.0, y=3.0, θ=1.57)` in the map frame
- Action Mapper constructs: `NavigateToPose(pose=kitchen_pose, frame_id='map')`

**Nav2 Path Planning**:
- Nav2 receives `NavigateToPose` goal
- **Costmap Construction**: Combines Isaac ROS depth images (3D obstacles) with 2D laser scans
- **Path Planning**: Uses algorithms like NavFn or Smac Planner to compute collision-free path
- **Obstacle Avoidance**: Dynamic Window Approach (DWA) or Regulated Pure Pursuit adjusts trajectory in real-time

**Feedback to VLA**:
- **Progress Updates**: Nav2 publishes feedback (`distance_remaining: 2.3m`)
- **Success**: Navigation completes → LLM continues to next action
- **Failure**: Path blocked → Nav2 reports failure → LLM replans with updated obstacle info

### Costmap Integration with Isaac ROS

Nav2's costmap merges multiple sensor inputs:
1. **Isaac ROS Depth Images**: 3D point clouds projected to 2D costmap layers
2. **Laser Scans**: 2D obstacles from LiDAR
3. **Inflation Layer**: Expands obstacles by robot radius + safety margin

**Data Flow**:
```
Isaac ROS Depth Images
    ↓
Published to /depth/image_raw (ROS 2)
    ↓
Nav2 Voxel Layer subscribes
    ↓
Projects 3D points to 2D costmap
    ↓
Costmap marks occupied cells
    ↓
Path Planner avoids occupied cells
    ↓
Collision-free path to kitchen
```

**Adaptive Replanning**:
- If path becomes blocked during execution (e.g., person walks into path), Nav2 replans dynamically
- If replanning fails repeatedly, Nav2 returns `FAILURE` result
- LLM receives failure feedback and can:
  - Request alternate route: "Navigate around obstacle"
  - Change goal: "Search for cup in living room instead"
  - Ask user for help: "Kitchen path is blocked. Please clear the way."

<!-- /chunk -->

<!-- chunk:complete-architecture -->

## Complete Autonomous Humanoid Architecture

The following diagram shows the **full stack** for autonomous humanoid robotics, integrating all four modules:

```
┌───────────────────────────────────────────────────────────────┐
│  Layer 5: User Interface                                      │
│  • Voice commands (microphone input)                          │
│  • Text commands (keyboard, app)                              │
│  • GUI (rviz2, web dashboard)                                 │
└───────────────────────┬───────────────────────────────────────┘
                        ↓ (natural language commands)
┌───────────────────────────────────────────────────────────────┐
│  Layer 4: VLA Cognitive Layer (Module 4)                      │
│  • Speech Recognition (Whisper, Google STT, Azure)            │
│  • NLU / Intent Extraction (BERT, rule-based)                 │
│  • LLM Cognitive Planner (GPT-4, Claude, PaLM-E)              │
│  • Action Mapper (intent → ROS 2 action goals)                │
└───────────────────────┬───────────────────────────────────────┘
                        ↓ (ROS 2 action goals)
┌───────────────────────────────────────────────────────────────┐
│  Layer 3: Perception & Navigation (Module 3)                  │
│  • Isaac ROS: Visual SLAM, object detection, segmentation     │
│  • Nav2: Path planning, costmap, obstacle avoidance           │
└───────────────────────┬───────────────────────────────────────┘
         ↑ (sensor data) ↓ (action goals, perception results)
┌───────────────────────────────────────────────────────────────┐
│  Layer 2: ROS 2 Middleware (Module 1)                         │
│  • Topics (sensor_msgs, geometry_msgs, perception_msgs)       │
│  • Actions (NavigateToPose, Grasp, DetectObjects)             │
│  • Services (speech recognition, semantic mapping)            │
│  • ROS 2 Control (joint trajectories, velocity commands)      │
└───────────────────────┬───────────────────────────────────────┘
         ↑ (raw data)   ↓ (control commands)
┌───────────────────────────────────────────────────────────────┐
│  Layer 1: Robot Hardware                                      │
│  • Sensors: Cameras, LiDAR, IMU, microphone, force/torque    │
│  • Actuators: Motors (joints), grippers, end effectors        │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│  Layer 0: Digital Twin (Module 2)                             │
│  • Gazebo simulation (physics, sensors)                       │
│  • Unity simulation (photorealistic rendering)                │
│  • Testing: Validate VLA pipelines before hardware deployment │
└───────────────────────────────────────────────────────────────┘
```

### Data Flow Paths

**Upward (Sensing)**:
1. Sensors capture data (camera images, LiDAR scans, audio)
2. ROS 2 topics publish sensor data
3. Isaac ROS processes perception (localization, detection, segmentation)
4. VLA Cognitive Planner subscribes to perception results, updates context

**Downward (Acting)**:
1. VLA Cognitive Planner generates action sequence
2. Action Mapper creates ROS 2 action goals
3. Nav2 / Manipulation controllers receive goals
4. ROS 2 Control translates to low-level commands (joint velocities, torques)
5. Hardware actuators execute motion

**Horizontal (Perception ↔ Navigation)**:
- Isaac ROS depth images → Nav2 costmap (obstacle detection)
- Nav2 localization ↔ Isaac ROS Visual SLAM (sensor fusion)

**Layer 0 (Digital Twin)** enables **safe testing**: the complete VLA pipeline runs in Gazebo/Unity simulation before deploying to physical hardware. This validates speech recognition, LLM planning, and action execution without risk of hardware damage.

<!-- /chunk -->

<!-- chunk:capstone-example -->

## Capstone Example: Kitchen Cup Retrieval

Let's walk through the complete execution of **"Go to the kitchen and bring me a cup"** with detailed step-by-step integration across all modules.

### Step 1: Speech Input & Recognition

**User**: "Go to the kitchen and bring me a cup"

**Microphone Array** (Hardware): Captures audio, beamforms toward speaker

**Speech Recognition** (Chapter 2, Module 4):
- Whisper model processes audio on GPU
- Transcription: `"Go to the kitchen and bring me a cup"`
- Publishes to `/speech/transcription` topic
- **Latency**: ~500ms

### Step 2: LLM Cognitive Planning

**NLU** (Chapter 2): Classifies as multi-step task → route to LLM Planner

**LLM Context** (Chapter 3, from Isaac ROS + Nav2):
```
Current Robot State:
- Location: living_room (x=1.2, y=0.8)
- Visible Objects: [table, chair] (no cup detected)
- Available Actions: navigate, detect_object, grasp, place, hand_over
```

**LLM Reasoning** (Chain-of-Thought):
```
"To bring the user a cup, I need to:
1. Navigate to the kitchen (where cups are typically stored)
2. Detect the cup's location using vision
3. Grasp the cup
4. Navigate back to the user
5. Hand over the cup"
```

**LLM Output**:
```
Action Sequence:
1. navigate(kitchen)
2. detect_object(cup)
3. grasp(cup)
4. navigate(user_location)
5. hand_over(cup)
```

**Latency**: ~2s (LLM inference)

### Step 3: Action Execution with Feedback Loop

#### Sub-Task 1: Navigate to Kitchen

**Action Mapper**: `navigate(kitchen)` → `NavigateToPose(x=5.0, y=3.0, θ=1.57, frame='map')`

**Nav2** (Module 3):
- Loads costmap (Isaac ROS depth + laser scans)
- Plans path avoiding obstacles
- Publishes velocity commands via ROS 2 Control (Module 1)

**Result**: `SUCCESS` (robot arrives at kitchen)

**Feedback**: LLM context updates: `Current location: kitchen`

---

#### Sub-Task 2: Detect Cup

**Action Mapper**: `detect_object(cup)` → `DetectObjects(class_id="cup")`

**Isaac ROS** (Module 3):
- Runs DOPE object detection on camera images
- Detects cup at `(x=2.3, y=1.1, z=0.8)` in robot base frame

**Result**: `SUCCESS` (cup detected)

**Feedback**: LLM context updates: `Visible objects: [cup (x=2.3, y=1.1, z=0.8)]`

**Failure Scenario** (demonstrates adaptive replanning):
- **Result**: `FAILURE` (cup not detected in current view)
- **LLM Replanning**: "Cup not found in kitchen. User mentioned 'kitchen,' but cup may be on counter, table, or shelf. Plan: search kitchen systematically by rotating camera to scan different areas."
- **Alternative**: Ask user: "I don't see a cup in the kitchen. Where should I look?"

---

#### Sub-Task 3: Grasp Cup

**Action Mapper**: `grasp(cup)` → `Grasp(target_pose=(2.3, 1.1, 0.8))`

**Manipulation Controller** (Module 1, ROS 2 Control):
- Computes inverse kinematics to reach target
- Opens gripper
- Approaches cup along safe trajectory
- Closes gripper
- Confirms grasp via force/torque sensor feedback

**Result**: `SUCCESS` (cup grasped securely)

**Feedback**: Tactile sensor confirms object in gripper

---

#### Sub-Task 4: Navigate Back to User

**Action Mapper**: `navigate(user_location)` → `NavigateToPose(x=1.2, y=0.8, θ=0.0, frame='map')`

**Nav2** (Module 3):
- Plans return path
- Moves carefully (holding fragile cup)
- Adjusts velocity profile for smooth motion

**Result**: `SUCCESS` (arrived at user)

---

#### Sub-Task 5: Hand Over Cup

**Action Mapper**: `hand_over(cup)` → `HandOver()`

**Manipulation Controller** (Module 1):
- Extends arm toward user
- Waits for user to grasp cup (force sensor detects pull)
- Opens gripper to release cup

**Result**: `SUCCESS` (task complete)

**User Feedback**: "Thank you!" (optional: user can rate task success, improving future LLM planning via reinforcement learning)

### Integration Summary

This capstone example demonstrates:
- **ROS 2 (Module 1)**: Action interfaces, control framework, communication
- **Digital Twin (Module 2)**: Test scenario in Gazebo before hardware deployment
- **Isaac ROS (Module 3)**: Object detection, localization, depth for costmap
- **Nav2 (Module 3)**: Collision-free navigation, path planning
- **VLA (Module 4)**: Speech recognition, LLM planning, action mapping, feedback integration

**Every system component** contributes to achieving the high-level goal: understanding natural language, reasoning about tasks, perceiving the environment, navigating safely, and executing precise manipulation—true **autonomous humanoid behavior**.

<!-- /chunk -->

<!-- chunk:course-conclusion -->

## Course Conclusion

Congratulations! You've completed the **Physical AI for Autonomous Humanoid Robots** course, mastering the complete technology stack:

### Module 1: ROS 2 Foundations
- Communication infrastructure (topics, actions, services)
- ROS 2 Control for hardware abstraction
- Lifecycle management and real-time execution

### Module 2: Digital Twins
- Simulation environments (Gazebo, Unity)
- Testing VLA systems safely before hardware deployment
- Physics-based validation of motion planning

### Module 3: NVIDIA Isaac ROS & Nav2
- Visual SLAM for localization
- Object detection and semantic segmentation
- Nav2 path planning and obstacle avoidance
- Costmap integration for collision-free navigation

### Module 4: Vision-Language-Action Systems
- Voice-to-action pipelines (speech recognition, NLU, action mapping)
- LLM-based cognitive planning (CoT, few-shot, ReAct)
- Adaptive replanning with perception feedback
- End-to-end integration across all modules

### The Complete Stack

You now understand how **humanoid robots achieve autonomous behavior** by integrating:
1. **Sensing**: Cameras, LiDAR, IMUs, microphones capture environmental data
2. **Perception**: Isaac ROS processes sensor data into semantic understanding
3. **Cognition**: LLMs reason about tasks, decompose goals, plan actions
4. **Navigation**: Nav2 plans collision-free paths using perception-derived costmaps
5. **Control**: ROS 2 Control executes low-level motion commands
6. **Actuation**: Motors and grippers perform physical actions
7. **Feedback**: Perception results inform replanning, enabling adaptive behavior

### Future Directions

To continue your journey in autonomous robotics, explore:
- **Multi-Agent Coordination**: Multiple robots collaborating on shared tasks
- **Continual Learning**: Robots improving from experience through reinforcement learning
- **Safety & Ethics**: Collision avoidance, fail-safe mechanisms, human-robot interaction safety
- **Real-World Deployment**: Transitioning from simulation (Digital Twins) to physical hardware

Build your own autonomous humanoid systems, experiment with simulation, and contribute to the future of intelligent robotics!

<!-- /chunk -->

<!-- chunk:key-takeaways -->

## Key Takeaways

- **Complete VLA pipeline** integrates seven stages: speech input, recognition, intent extraction, LLM planning, action mapping, execution, and perception feedback.
- **Isaac ROS** (Module 3) provides environmental understanding through Visual SLAM, object detection, segmentation, and depth estimation, enabling grounded LLM planning.
- **Nav2** (Module 3) handles collision-free navigation with costmaps derived from Isaac ROS depth images, supporting adaptive path replanning.
- **Complete autonomous architecture** spans six layers: User Interface, VLA Cognitive Layer (Module 4), Perception/Navigation (Module 3), ROS 2 Middleware (Module 1), Hardware, and Digital Twin (Module 2).
- **Capstone example** ("bring me a cup") demonstrates all four modules working together with adaptive replanning based on execution feedback.
- **Feedback loops** between perception and planning enable robust autonomous behavior—robots adapt to environmental changes, execution failures, and novel scenarios.
- **Digital Twins** (Module 2) allow safe testing of the complete VLA system in simulation before hardware deployment, reducing development risk.

<!-- /chunk -->

---

## Code Example: Feedback Loop Integration

```python
# Conceptual Feedback Loop Integration for VLA System
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://docs.ros.org/en/humble/

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from isaac_ros_msgs.action import DetectObjects
from manipulation_msgs.action import Grasp

class VLAFeedbackLoop(Node):
    def __init__(self, cognitive_planner, action_mapper):
        super().__init__('vla_feedback_loop')
        self.planner = cognitive_planner
        self.mapper = action_mapper

        # ROS 2 action clients for Nav2, Isaac ROS, manipulation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.perception_client = ActionClient(self, DetectObjects, 'detect_objects')
        self.grasp_client = ActionClient(self, Grasp, 'grasp')

    def execute_task(self, user_command):
        """Execute multi-step task with adaptive feedback loop"""

        # Get initial environmental context from Isaac ROS perception
        context = self.get_perception_context()

        # LLM cognitive planning: decompose task into action sequence
        action_sequence = self.planner.plan(user_command, context)

        # Execute each action with feedback-driven replanning
        for i, action in enumerate(action_sequence):
            self.get_logger().info(f'Executing action {i+1}/{len(action_sequence)}: {action}')

            # Map abstract action to ROS 2 action goal
            ros_goal = self.mapper.to_ros_action(action)

            # Send goal to appropriate action server
            if action['type'] == 'navigate':
                future = self.nav_client.send_goal_async(ros_goal)
            elif action['type'] == 'detect':
                future = self.perception_client.send_goal_async(ros_goal)
            elif action['type'] == 'grasp':
                future = self.grasp_client.send_goal_async(ros_goal)

            # Wait for action completion
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f'Action {action} rejected by server')
                return self.handle_rejection(user_command, context, action)

            # Get action result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            # Feedback-driven replanning
            if result.success:
                # Update context with execution result
                context = self.update_context(context, action, result)
                self.get_logger().info(f'Action succeeded. Continuing...')
                continue  # Proceed to next action
            else:
                # Failure: trigger adaptive replanning
                self.get_logger().warn(f'Action failed: {result.message}')

                # Get updated perception context (environment may have changed)
                context = self.get_perception_context()

                # LLM replans with updated context and failure info
                action_sequence = self.planner.replan(
                    user_command,
                    context,
                    failed_action=action,
                    failure_reason=result.message
                )

                # Retry with new plan
                self.get_logger().info(f'Replanned. New sequence: {action_sequence}')
                # Continue loop with updated action_sequence

        return "Task completed successfully"

    def get_perception_context(self):
        """Query Isaac ROS for current environmental state"""
        # In practice: subscribe to /detected_objects, /visual_slam/pose topics
        return {
            'location': 'living_room',
            'objects': ['cup (x=2.3, y=1.1)', 'table (x=2.0, y=1.0)'],
            'obstacles': 'Nav2 costmap'
        }

    def update_context(self, context, action, result):
        """Update context with action execution results"""
        if action['type'] == 'navigate':
            context['location'] = action['target']
        elif action['type'] == 'detect':
            context['objects'] = result.detected_objects
        return context

    def handle_rejection(self, command, context, action):
        """Handle action server rejection (e.g., invalid goal)"""
        self.get_logger().error(f'Action {action} rejected. Requesting user clarification.')
        # In practice: use speech synthesis to ask user for help
        return f"Cannot execute {action}. Please provide more information."


def main(args=None):
    rclpy.init(args=args)

    # Initialize LLM cognitive planner and action mapper
    from cognitive_planner import CognitivePlanner  # Conceptual import
    from action_mapper import ActionMapper

    planner = CognitivePlanner(model='gpt-4')
    mapper = ActionMapper()

    # Create VLA feedback loop node
    vla_node = VLAFeedbackLoop(planner, mapper)

    # Execute task with user command
    user_command = "Go to the kitchen and bring me a cup"
    result = vla_node.execute_task(user_command)

    vla_node.get_logger().info(f'Task result: {result}')

    vla_node.destroy_node()
    rclpy.shutdown()
```

This code demonstrates the complete feedback loop: LLM planning, ROS 2 action execution (Nav2, Isaac ROS, manipulation), result handling, and adaptive replanning on failure. The integration of perception context (Isaac ROS), navigation (Nav2), control (ROS 2 actions), and cognitive planning (LLM) achieves robust autonomous behavior across all four modules.
