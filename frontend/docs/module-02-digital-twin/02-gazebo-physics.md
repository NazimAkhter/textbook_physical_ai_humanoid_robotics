---
id: gazebo-physics
title: "Physics Simulation with Gazebo"
sidebar_label: "Gazebo Physics"
sidebar_position: 2
description: "Learn Gazebo architecture, physics engines, and ROS 2 integration for humanoid robot simulation."
keywords: ["Gazebo", "physics simulation", "ODE", "Bullet", "DART", "ROS 2", "gz-ros2-control", "SDF"]
learning_objectives:
  - "Describe Gazebo architecture and physics engine options"
  - "Explain world modeling concepts (gravity, friction, contacts)"
  - "Understand URDF-to-SDF conversion and ROS 2 integration"
---

# Physics Simulation with Gazebo

:::note Prerequisites
This chapter builds on URDF concepts from Module 1. Familiarity with robot description formats and ROS 2 middleware patterns will help you understand how simulation integrates with the robotics stack.
:::

<!-- chunk:gazebo-architecture -->

## Gazebo Architecture

**Gazebo** is an open-source robotics simulator that provides physics simulation, sensor modeling, and visualization in a single integrated environment. For humanoid robotics, Gazebo enables testing balance controllers, locomotion algorithms, and manipulation tasks before deploying to physical hardware.

The architecture follows a modular design with three core components working together:

**Physics Server**: The computational engine that advances simulation time, computing forces, collisions, and joint dynamics at each timestep. The server maintains the world state and handles all physics calculations.

**Rendering Engine**: Generates visual output for cameras (both simulated sensors and human observation). Uses OGRE for 3D graphics, supporting shadows, reflections, and material properties.

**Transport Layer**: A publish-subscribe messaging system (similar to ROS topics) that connects simulation components. Sensors publish data, plugins subscribe to control commands, and external tools can monitor simulation state.

:::info Diagram: Gazebo Architecture

**Type**: architecture

**Description**: Block diagram showing Gazebo's modular components and their interactions.

**Key Elements**:
- Physics Server block (contains physics engine, collision detection, solver)
- Rendering Engine block (OGRE, cameras, GUI)
- Transport Layer connecting all components
- Plugin interfaces for sensors, controllers, and world manipulation
- External connections: ROS 2 bridge, command-line tools

:::

<!-- /chunk -->

<!-- chunk:physics-engines -->

## Physics Engine Options

Gazebo supports multiple physics engines, each with different accuracy-performance trade-offs. The physics engine determines how rigid body dynamics, collisions, and contacts are computed.

| Engine | Strengths | Considerations | Best For |
|--------|-----------|----------------|----------|
| **ODE** | Stable, well-tested, fast | Less accurate friction | General robotics, rapid prototyping |
| **Bullet** | Good collision detection, GPU support | Contact stability varies | Manipulation, many-body scenes |
| **DART** | Accurate dynamics, Featherstone solver | Higher computational cost | Humanoid balance, precise dynamics |
| **Simbody** | Biomechanics focus, constraints | Specialized use cases | Human modeling, bio-inspired robots |

For humanoid balance control, **DART** (Dynamic Animation and Robotics Toolkit) often provides the best dynamics accuracy because it uses the Featherstone algorithm—the same mathematical approach used in analytical robotics. ODE remains popular for its speed and stability during initial development.

The choice isn't permanent: you can switch physics engines by changing a configuration parameter, then validate that your controllers behave consistently across engines.

<!-- /chunk -->

<!-- chunk:world-modeling -->

## World Modeling

A Gazebo **world** defines the environment where your robot operates: ground planes, obstacles, lighting, and the physics parameters governing interactions.

**Gravity**: Defined as a 3D vector (typically `[0, 0, -9.81]` m/s²). For humanoid testing, accurate gravity is essential—even small errors compound during locomotion.

**Friction**: Determines how surfaces resist sliding. Contact friction uses coefficients (μ₁, μ₂) for each surface pair. Humanoid foot-ground friction directly affects walking stability.

**Contact Dynamics**: When bodies collide, the physics engine computes contact points, normal forces, and friction forces. Parameters like `max_contacts`, `soft_cfm` (constraint force mixing), and `soft_erp` (error reduction parameter) tune this behavior.

```xml
<!-- NOTE: Illustrative SDF world fragment, not production configuration -->
<world name="humanoid_test_world">
  <gravity>0 0 -9.81</gravity>
  <physics type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <model name="ground_plane">
    <static>true</static>
    <link name="ground">
      <collision name="collision">
        <surface>
          <friction>
            <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</world>
```

The `max_step_size` parameter is critical: smaller timesteps increase accuracy but slow simulation. For humanoid control running at 1 kHz, a 1ms physics timestep maintains synchronization between controller and simulation.

<!-- /chunk -->

<!-- chunk:urdf-sdf-conversion -->

## URDF to SDF Conversion

Your robot's URDF from Module 1 must be converted to **SDF** (Simulation Description Format) for Gazebo. While URDF describes robot structure, SDF extends this with simulation-specific properties: physics parameters, sensor definitions, and plugin attachments.

:::info Diagram: URDF-to-SDF Conversion Flow

**Type**: data-flow

**Description**: Pipeline showing how robot descriptions transform for simulation.

**Key Elements**:
- Input: URDF file (links, joints, transmissions)
- Conversion step: `gz sdf -p robot.urdf > robot.sdf`
- Augmentation: Add Gazebo-specific tags (`<gazebo>`, `<sensor>`, `<plugin>`)
- Output: Complete SDF model ready for simulation
- Arrows showing data flow and transformation points

:::

The conversion process:

1. **Parse URDF**: Extract links, joints, visual geometry, collision geometry
2. **Generate SDF**: Create equivalent SDF structure with physics properties
3. **Add Gazebo Extensions**: Attach sensors, plugins, and simulation parameters

In practice, many robotics projects maintain a URDF with embedded Gazebo extension tags that provide simulation hints. The ros2_control framework handles the bridge between your control code and simulated actuators.

<!-- /chunk -->

<!-- chunk:ros2-control-integration -->

## ROS 2 Integration with gz-ros2-control

The **gz-ros2-control** package bridges ROS 2 controllers with Gazebo actuators, enabling the same control code to run in simulation and on hardware.

The integration pattern follows ros2_control architecture:

1. **Hardware Interface**: `gz-ros2-control` provides a `GazeboSystem` hardware interface that translates ROS 2 commands to Gazebo joint commands
2. **Controller Manager**: Loads and manages controllers (joint trajectory, velocity, effort)
3. **Control Loop**: Controllers read joint states from simulation, compute commands, and write back—just as they would with real hardware

This means your balance controller developed in simulation uses the same ROS 2 interfaces it will use on the physical humanoid. The only change at deployment: swap the hardware interface from `GazeboSystem` to your actual motor drivers.

**Topic Bridge**: The `ros_gz_bridge` package connects Gazebo transport topics to ROS 2 topics. Simulated sensor data (cameras, IMU, force-torque) publishes to ROS 2 topics that perception nodes consume, unaware they're receiving simulated data.

<!-- /chunk -->

## Key Takeaways

- **Gazebo architecture** combines physics server, rendering engine, and transport layer in a modular design supporting multiple physics engines.
- **Physics engine choice** affects simulation accuracy and speed—DART excels for humanoid dynamics, ODE for rapid iteration.
- **World modeling** defines gravity, friction, and contact parameters that directly impact humanoid balance and locomotion behavior.
- **URDF-to-SDF conversion** transforms robot descriptions for simulation, adding physics and sensor properties.
- **gz-ros2-control** enables the same ROS 2 controllers to run in simulation and on hardware through a common hardware interface abstraction.
- **Same interfaces, different backend**: The ros2_control architecture means simulation testing validates the actual control code you'll deploy.

---

*Next: [Visualization and Interaction with Unity](./03-unity-visualization.md) — Explore high-fidelity rendering and human-robot interaction scenarios.*
