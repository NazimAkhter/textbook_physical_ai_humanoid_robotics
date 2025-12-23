---
id: urdf-humanoids
title: "Understanding URDF for Humanoid Robots"
sidebar_label: "URDF Basics"
sidebar_position: 4
description: "Learn how URDF defines humanoid robot structure through links, joints, and transmissions for simulation and control."
keywords: ["URDF", "humanoid robots", "links", "joints", "transmissions", "robot description", "digital twins"]
learning_objectives:
  - "Identify URDF structural elements (links, joints, transmissions)"
  - "Explain humanoid kinematic modeling concepts"
  - "Connect URDF to simulation and digital twin applications"
---

# Understanding URDF for Humanoid Robots

<!-- chunk:urdf-overview -->

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML-based format that describes the physical structure of a robot. It defines the robot's **links** (rigid bodies), **joints** (connections between links), and properties like mass, inertia, and visual geometry.

For humanoid robots, URDF serves as the structural blueprint that enables:
- **Simulation**: Physics engines like Gazebo use URDF to simulate robot dynamics
- **Control**: Motion planners and controllers use URDF for kinematics calculations
- **Visualization**: Tools like RViz render the robot model for debugging
- **Digital twins**: The same URDF describes both the simulated and physical robot

Think of URDF as the architectural drawing of your robot—it doesn't define behavior (that's the software), but it precisely describes the physical structure that software must control.

<!-- /chunk -->

## URDF Structure

<!-- chunk:link-concept -->

### Links: Rigid Bodies

A **link** represents a rigid body in the robot—something that doesn't deform. In a humanoid, typical links include the torso, upper arm, forearm, thigh, shin, and foot.

Each link defines:
- **Visual geometry**: How the link appears (mesh or primitive shapes)
- **Collision geometry**: Simplified shape for physics collision detection
- **Inertial properties**: Mass, center of mass, and inertia tensor

```xml title="URDF Link Definition"
<!-- NOTE: This is an illustrative example, not production code -->
<link name="left_upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.28"/>
    </geometry>
    <material name="gray"/>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.28"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.2"/>
    <inertia ixx="0.01" ixy="0" ixz="0"
             iyy="0.01" iyz="0" izz="0.002"/>
  </inertial>
</link>
```

<!-- /chunk -->

<!-- chunk:joint-concept -->

### Joints: Connections Between Links

A **joint** connects two links and defines how they can move relative to each other. URDF supports several joint types:

| Joint Type | Motion | Humanoid Example |
|------------|--------|------------------|
| **revolute** | Rotation around axis (limited range) | Elbow, knee, shoulder |
| **continuous** | Unlimited rotation | Wheel (not typical for humanoids) |
| **prismatic** | Linear sliding | Telescoping mechanism |
| **fixed** | No motion | Sensor mount to torso |

```xml title="URDF Joint Definition"
<!-- NOTE: This is an illustrative example, not production code -->
<joint name="left_elbow" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_forearm"/>
  <origin xyz="0 0 -0.28" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="50" velocity="3.0"/>
</joint>
```

Key joint properties:
- **parent/child**: Which links this joint connects
- **origin**: Position and orientation of joint frame
- **axis**: Rotation or translation axis
- **limit**: Range of motion, maximum effort (torque/force), and velocity

:::info Diagram: Link-Joint Relationship

**Type**: hierarchy

**Description**: Visual showing how links connect through joints, with parent-child relationships forming a kinematic tree.

**Key Elements**:
- Parent link with coordinate frame
- Joint axis and origin offset
- Child link attached through joint
- Arrows indicating allowed motion direction

:::

<!-- /chunk -->

<!-- chunk:humanoid-kinematics -->

## Humanoid Kinematic Structure

:::info Diagram: Humanoid Joint Hierarchy

**Type**: hierarchy

**Description**: Tree structure of a humanoid robot showing the kinematic chain from base to end effectors.

**Key Elements**:
- Base link (pelvis/torso) at root
- Branching to left/right legs and arms
- Head chain branching from torso
- End effectors: hands, feet, head
- Joint labels at each connection

:::

A humanoid robot forms a **kinematic tree** with the torso (or pelvis) as the root:

- **Torso** → branches to head, left arm, right arm, left leg, right leg
- **Arm chain**: shoulder → upper arm → elbow → forearm → wrist → hand
- **Leg chain**: hip → thigh → knee → shin → ankle → foot

Each chain is a sequence of links connected by joints. The total degrees of freedom (DOF) equals the sum of all joint DOFs—a typical humanoid has 20-40 DOF.

<!-- /chunk -->

<!-- chunk:transmission-concept -->

### Transmissions: Actuator Interfaces

**Transmissions** connect joints to actuators, defining the relationship between motor effort and joint motion. While basic URDF models may omit transmissions, they're important for:
- **Gear ratios**: Motor rotation to joint rotation mapping
- **Control interfaces**: Telling ros2_control how to command each joint

Transmissions bridge the gap between URDF's kinematic description and the actual control system—they answer "how does motor torque become joint motion?"

<!-- /chunk -->

<!-- chunk:urdf-simulation -->

## URDF and Digital Twins

URDF is the foundation of robot **digital twins**—virtual replicas that mirror physical robots. The same URDF file:

1. **Defines the simulated robot** in Gazebo or NVIDIA Isaac (Module 2-3)
2. **Configures the real robot's** kinematic model for motion planning
3. **Enables visualization** in RViz for debugging

This consistency means algorithms developed in simulation transfer more reliably to hardware. When your simulated humanoid walks using URDF-based physics, and your real humanoid uses the same URDF for control, you've minimized one source of simulation-to-reality gap.

<!-- /chunk -->

## Key Takeaways

- **URDF** is an XML format describing robot structure—the blueprint for simulation, control, and visualization.
- **Links** represent rigid bodies with visual, collision, and inertial properties.
- **Joints** connect links and define motion types: revolute for rotation (most humanoid joints), prismatic for sliding, fixed for rigid connections.
- **Humanoid kinematics** form a tree structure branching from the torso to limbs, with each chain contributing degrees of freedom.
- **Transmissions** connect joints to actuators, defining how motor effort translates to joint motion.
- **Digital twins** rely on URDF to ensure consistency between simulated and physical robots, enabling simulation-to-reality transfer.
- **ROS 2 nodes** (see [Chapter 2](./02-core-primitives.md)) use URDF-defined joint names when publishing commands and subscribing to state.

---

*This concludes Module 1. You now understand ROS 2's role as robotic middleware, its core communication primitives, how Python integrates via rclpy, and how URDF describes robot structure. In Module 2, we'll explore how these concepts come together in simulation environments and digital twins.*
