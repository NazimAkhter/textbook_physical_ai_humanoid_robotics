---
id: core-primitives
title: "ROS 2 Core Primitives – Nodes, Topics, and Services"
sidebar_label: "Core Primitives"
sidebar_position: 2
description: "Learn the building blocks of ROS 2 communication: Nodes, Topics, Services, and Actions with humanoid robot examples."
keywords: ["ROS 2", "nodes", "topics", "services", "actions", "pub/sub", "humanoid control"]
learning_objectives:
  - "Explain the ROS 2 node lifecycle"
  - "Differentiate between Topics, Services, and Actions"
  - "Illustrate data flow patterns in humanoid robot systems"
---

# ROS 2 Core Primitives – Nodes, Topics, and Services

:::note Prerequisites
This chapter builds on concepts from [Chapter 1: ROS 2 as the Robotic Nervous System](./01-ros2-overview.md). Understanding ROS 2's role as middleware will help you appreciate how these primitives work together.
:::

<!-- chunk:node-concept -->

## Nodes: The Building Blocks

A **node** is an independent process that performs a specific function within the robot system. Each node has a well-defined responsibility—one might process camera images, another might control a single joint, and another might plan whole-body trajectories.

In a humanoid robot, you might have nodes for:
- **Perception**: Processing depth camera data, detecting obstacles
- **State estimation**: Fusing IMU and joint encoder data to estimate body pose
- **Motion planning**: Computing collision-free paths for arms
- **Balance control**: Generating torque commands to maintain stability
- **Joint drivers**: Interfacing with hardware actuators

### Node Lifecycle

ROS 2 nodes follow a **managed lifecycle** with defined states:

1. **Unconfigured**: Node created but not ready
2. **Inactive**: Configured but not processing
3. **Active**: Fully operational, processing data
4. **Finalized**: Shutting down

```python title="Node Lifecycle States"
# NOTE: This is an illustrative example, not production code
# Lifecycle states for a managed ROS 2 node:
# - unconfigured: Node exists but hasn't loaded parameters
# - inactive: Parameters loaded, ready to activate
# - active: Processing callbacks, publishing/subscribing
# - finalized: Cleaning up before destruction
```

This lifecycle enables orderly startup and shutdown of complex robot systems—you can bring up perception nodes first, then controllers, ensuring dependencies are satisfied.

<!-- /chunk -->

## Communication Patterns

<!-- chunk:topic-concept -->

### Topics: Continuous Data Streams

**Topics** implement a publish-subscribe pattern for continuous data streams. A node **publishes** messages to a named topic; any number of nodes can **subscribe** to receive those messages. Publishers and subscribers don't know about each other—they only know the topic name and message type.

**When to use Topics**:
- Sensor data that flows continuously (camera frames, IMU readings)
- State information that changes over time (joint positions, robot pose)
- Commands that update frequently (velocity setpoints)

**Humanoid Example**: A force-torque sensor at the robot's ankle continuously publishes readings to `/left_ankle/force_torque`. The balance controller subscribes to this topic to detect ground contact and adjust stability.

```python title="Publisher/Subscriber Pattern"
# NOTE: This is an illustrative example, not production code
# Publisher creates messages and sends to topic
publisher = node.create_publisher(JointState, '/joint_states', 10)
publisher.publish(joint_state_msg)

# Subscriber receives messages via callback
subscriber = node.create_subscription(
    JointState, '/joint_states', callback_function, 10)
```

<!-- /chunk -->

<!-- chunk:service-concept -->

### Services: Request-Response

**Services** implement a synchronous request-response pattern. A **client** sends a request and waits for a response from the **server**. Unlike topics, services are one-to-one and blocking.

**When to use Services**:
- Configuration queries (get current parameter values)
- Discrete actions with immediate results (enable motor, get sensor calibration)
- Operations that need confirmation of completion

**Humanoid Example**: Before walking, the locomotion controller calls a service `/get_foot_contact_state` to determine which foot is currently supporting the robot.

| Pattern | Communication | Use Case |
|---------|---------------|----------|
| Topics | Many-to-many, async | Continuous sensor data |
| Services | One-to-one, sync | Discrete queries/commands |

<!-- /chunk -->

<!-- chunk:action-concept -->

### Actions: Long-Running Tasks

**Actions** extend the service pattern for long-running tasks that need progress feedback. A client sends a goal, receives periodic feedback during execution, and gets a final result. Actions also support cancellation.

**When to use Actions**:
- Motion execution (move arm to target pose)
- Navigation (walk to waypoint)
- Any task that takes significant time and benefits from progress updates

**Humanoid Example**: Walking to a goal position uses an action. The planner sends the goal, receives feedback about current position and progress, and can cancel if an obstacle appears.

<!-- /chunk -->

## Data Flow in Humanoid Systems

<!-- chunk:humanoid-data-flow -->

:::info Diagram: Communication Patterns Comparison

**Type**: comparison

**Description**: Visual comparison of Topics (one-to-many broadcast), Services (request-response), and Actions (goal-feedback-result) patterns.

**Key Elements**:
- Topics: Multiple arrows from publisher to multiple subscribers
- Services: Bidirectional arrow between client and server
- Actions: Three-part flow showing goal, feedback stream, and result

:::

In a humanoid robot, these patterns combine to create complex behaviors:

1. **Sensor nodes** publish to topics: joint encoders → `/joint_states`, cameras → `/camera/image_raw`
2. **Perception nodes** subscribe to sensor topics, publish processed results: `/detected_objects`, `/body_pose`
3. **Planning nodes** offer action servers: `/walk_to_pose`, `/pick_object`
4. **Control nodes** subscribe to command topics, query services for state, publish actuator commands

:::info Diagram: Humanoid Data Flow

**Type**: data-flow

**Description**: Data flow diagram showing how sensor data flows through perception, planning, and control layers in a humanoid robot.

**Key Elements**:
- Sensors layer: Joint encoders, IMU, cameras, force sensors
- Perception layer: State estimation, object detection
- Planning layer: Motion planning, gait generation
- Control layer: Balance control, joint controllers
- Arrows showing topic subscriptions and publications between layers

:::

<!-- /chunk -->

<!-- chunk:executor-concept -->

## Executors: Managing Callbacks

The **executor** is the engine that runs node callbacks. When messages arrive on subscribed topics, service requests come in, or action goals are received, the executor dispatches these events to the appropriate callback functions.

ROS 2 offers different executor types:
- **SingleThreadedExecutor**: Processes callbacks sequentially (simpler, deterministic)
- **MultiThreadedExecutor**: Processes callbacks in parallel (higher throughput)
- **StaticSingleThreadedExecutor**: Optimized for real-time with minimal overhead

For humanoid robots running real-time control loops, executor choice affects timing predictability. Critical control nodes often use dedicated single-threaded executors to guarantee deterministic callback timing.

<!-- /chunk -->

## Key Takeaways

- **Nodes** are independent processes with specific responsibilities; managed lifecycle enables orderly system startup.
- **Topics** provide publish-subscribe for continuous data streams—ideal for sensor data and state updates.
- **Services** offer synchronous request-response for discrete queries and commands.
- **Actions** handle long-running tasks with progress feedback and cancellation support.
- **Humanoid robots** combine all patterns: topics for sensor streams, services for state queries, actions for motion execution.
- **Executors** dispatch callbacks; choice of executor type affects real-time performance.
- **Python integration** via rclpy (covered in [Chapter 3](./03-rclpy-integration.md)) makes these primitives accessible to AI developers.

---

*Next: [Bridging Python AI Agents to ROS 2](./03-rclpy-integration.md) — Learn how to interface Python-based AI systems with ROS 2 using rclpy.*
