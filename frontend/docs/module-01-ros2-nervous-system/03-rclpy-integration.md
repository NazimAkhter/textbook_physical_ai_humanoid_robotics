---
id: rclpy-integration
title: "Bridging Python AI Agents to ROS 2 using rclpy"
sidebar_label: "rclpy Integration"
sidebar_position: 3
description: "Learn how Python-based AI agents interface with ROS 2 using rclpy for publishing, subscribing, and robot control."
keywords: ["rclpy", "Python", "ROS 2", "AI agents", "publisher", "subscriber", "robot control"]
learning_objectives:
  - "Describe rclpy architecture and execution model"
  - "Demonstrate publishing and subscribing patterns in Python"
  - "Explain how AI agents can command robot motion via ROS 2"
---

# Bridging Python AI Agents to ROS 2 using rclpy

:::note Prerequisites
This chapter builds on [Chapter 1](./01-ros2-overview.md) (middleware concepts) and [Chapter 2](./02-core-primitives.md) (nodes, topics, services). Understanding these fundamentals is essential for effective rclpy usage.
:::

<!-- chunk:rclpy-overview -->

## What is rclpy?

**rclpy** (ROS Client Library for Python) is the official Python interface to ROS 2. It allows you to write ROS 2 nodes entirely in Python—creating publishers, subscribers, service clients, and action clients using familiar Python syntax.

For AI practitioners, rclpy is particularly valuable because:
- **Python ecosystem access**: Use NumPy, PyTorch, TensorFlow, and other ML libraries directly in ROS 2 nodes
- **Rapid prototyping**: Test AI algorithms quickly without C++ compilation
- **Integration flexibility**: Connect trained models to robot hardware through ROS 2's communication layer

<!-- /chunk -->

<!-- chunk:rclpy-architecture -->

## rclpy Architecture

rclpy wraps the underlying ROS Client Library (RCL), which is written in C. This layered architecture means:

1. **Python code** (your node) calls rclpy functions
2. **rclpy** translates these to RCL calls
3. **RCL** interfaces with the RMW (ROS Middleware) layer
4. **RMW** communicates through DDS

```python title="Basic rclpy Node Structure"
# NOTE: This is an illustrative example, not production code
import rclpy
from rclpy.node import Node

class MyAINode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        # Node is now registered with ROS 2 graph
        # Can create publishers, subscribers, etc.

def main():
    rclpy.init()            # Initialize ROS 2 context
    node = MyAINode()       # Create node instance
    rclpy.spin(node)        # Process callbacks until shutdown
    rclpy.shutdown()        # Clean up resources
```

The `rclpy.spin()` call is crucial—it blocks and continuously processes incoming messages, service requests, and timer callbacks. Without spinning, your node won't receive any data.

<!-- /chunk -->

## Publishing Data

<!-- chunk:publisher-pattern -->

To send data from a Python AI agent to other ROS 2 nodes, you create a **publisher**. This is how your AI model's outputs reach robot controllers.

```python title="Creating a Publisher"
# NOTE: This is an illustrative example, not production code
from geometry_msgs.msg import Twist

class MotionCommandNode(Node):
    def __init__(self):
        super().__init__('motion_commander')
        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,              # Message type
            '/cmd_vel',         # Topic name
            10                  # Queue size (QoS depth)
        )

    def send_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_publisher.publish(msg)
```

**AI Agent Example**: A vision-based navigation model processes camera images, determines the robot should turn left, and publishes a `Twist` message with angular velocity. The locomotion controller subscribes to `/cmd_vel` and executes the turn.

<!-- /chunk -->

## Subscribing to Data

<!-- chunk:subscriber-pattern -->

To receive sensor data or state information, you create a **subscriber** with a callback function that processes each incoming message.

```python title="Creating a Subscriber"
# NOTE: This is an illustrative example, not production code
from sensor_msgs.msg import JointState

class StateMonitorNode(Node):
    def __init__(self):
        super().__init__('state_monitor')
        self.subscription = self.create_subscription(
            JointState,              # Message type
            '/joint_states',         # Topic name
            self.joint_callback,     # Callback function
            10                       # Queue size
        )

    def joint_callback(self, msg):
        # Called every time a message arrives
        positions = msg.position
        velocities = msg.velocity
        # Feed to AI model, log, or process as needed
```

**AI Agent Example**: A learning-based controller subscribes to joint states, feeds the data through a neural network policy, and publishes torque commands. The callback runs at sensor rate, enabling reactive control.

<!-- /chunk -->

<!-- chunk:ai-integration -->

## Integrating AI Models

:::info Diagram: AI Agent to ROS 2 Integration

**Type**: architecture

**Description**: Data flow from ROS 2 sensors through a Python AI agent node back to robot actuators.

**Key Elements**:
- Input: Subscriber receiving sensor data (images, joint states, IMU)
- Processing: AI model (neural network, planner, decision system)
- Output: Publisher sending commands (joint torques, velocities, goals)
- Timer triggering control loop at fixed rate

:::

A typical AI agent node combines subscribers, publishers, and timers:

1. **Subscribe** to sensor topics (camera, joint states, force sensors)
2. **Buffer** recent data for temporal context
3. **Run inference** when new data arrives or at fixed intervals
4. **Publish** commands based on model output

For real-time performance, consider:
- **Timer-based control**: Use `create_timer()` to run the control loop at a fixed rate, rather than triggering on every sensor message
- **Data buffering**: Store recent observations for models that need temporal context
- **Asynchronous inference**: For expensive models, consider running inference in a separate thread to avoid blocking callbacks

<!-- /chunk -->

<!-- chunk:callback-pattern -->

## Callbacks and Execution

Callbacks are the heart of reactive ROS 2 programming. Understanding when they execute helps you design responsive AI agents:

- **Subscriber callbacks**: Run when a message arrives on the subscribed topic
- **Timer callbacks**: Run at the specified interval (e.g., every 10ms for 100Hz control)
- **Service callbacks**: Run when a service request is received

The executor manages callback scheduling. By default, rclpy uses a single-threaded executor, meaning callbacks run sequentially. For AI workloads, this is often sufficient—and simpler to reason about than multithreaded execution.

<!-- /chunk -->

## Key Takeaways

- **rclpy** is the Python interface to ROS 2, enabling AI practitioners to leverage the Python ML ecosystem within robot systems.
- **Node structure** follows a pattern: initialize rclpy, create node class, spin to process callbacks, shutdown cleanly.
- **Publishers** send data to topics—use them to output AI model predictions as robot commands.
- **Subscribers** receive data via callbacks—use them to feed sensor data into your AI models.
- **AI integration** typically combines subscribers (input), processing (inference), publishers (output), and timers (control rate).
- **Callbacks execute sequentially** by default; design your node's timing accordingly for real-time requirements.
- **Robot structure** defined in URDF ([Chapter 4](./04-urdf-humanoids.md)) determines what joints and sensors your rclpy nodes can control.

---

*Next: [Understanding URDF for Humanoid Robots](./04-urdf-humanoids.md) — Learn how robot structure is defined using URDF for simulation and control.*
