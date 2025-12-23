# Data Model: Module 2 – The Digital Twin (Gazebo & Unity)

**Date**: 2024-12-14
**Feature**: `002-digital-twin-gazebo-unity`
**Purpose**: Define content entities, relationships, and structural contracts

## Content Entities

### Module Entity

```yaml
Module:
  id: module-02-digital-twin
  title: "Module 2: The Digital Twin (Gazebo & Unity)"
  position: 2  # After Module 1
  description: "Learn how digital twins enable safe robotics development through physics simulation, visualization, and sensor modeling."
  prerequisites:
    - module-01-ros2-nervous-system
  chapters: [Chapter1, Chapter2, Chapter3, Chapter4]
  word_count_target: 2500-4000
```

### Chapter Entities

```yaml
Chapter1:
  id: digital-twins
  file: 01-digital-twins.md
  title: "Digital Twins in Robotics"
  sidebar_label: "Digital Twins"
  sidebar_position: 1
  word_target: 625-1000
  learning_objectives:
    - "Define digital twin and explain its role in robotics development"
    - "Identify 3+ benefits of simulation-driven development"
    - "Compare simulation fidelity levels and their trade-offs"
  key_concepts:
    - digital-twin-definition
    - simulation-benefits
    - fidelity-levels
    - simulation-ecosystem
  diagrams: 1-2
  code_snippets: 0

Chapter2:
  id: gazebo-physics
  file: 02-gazebo-physics.md
  title: "Physics Simulation with Gazebo"
  sidebar_label: "Gazebo Physics"
  sidebar_position: 2
  word_target: 625-1000
  learning_objectives:
    - "Describe Gazebo architecture and physics engine options"
    - "Explain world modeling concepts (gravity, friction, contacts)"
    - "Understand URDF-to-SDF conversion and ROS 2 integration"
  key_concepts:
    - gazebo-architecture
    - physics-engines
    - world-modeling
    - urdf-sdf-conversion
    - ros2-control-integration
  diagrams: 2
  code_snippets: 1-2
  prerequisites_from_module1:
    - URDF structure (Chapter 4)
    - ROS 2 topics/services (Chapter 2)

Chapter3:
  id: unity-visualization
  file: 03-unity-visualization.md
  title: "Visualization and Interaction with Unity"
  sidebar_label: "Unity Visualization"
  sidebar_position: 3
  word_target: 625-1000
  learning_objectives:
    - "Explain when and why to use Unity for robotics applications"
    - "Understand Unity-ROS integration concepts"
    - "Compare Gazebo and Unity for different use cases"
  key_concepts:
    - unity-robotics-visualization
    - rendering-vs-physics
    - human-robot-interaction
    - ros-tcp-connector
    - gazebo-unity-comparison
  diagrams: 1-2
  code_snippets: 0-1

Chapter4:
  id: sensor-simulation
  file: 04-sensor-simulation.md
  title: "Sensor Simulation for Perception"
  sidebar_label: "Sensor Simulation"
  sidebar_position: 4
  word_target: 625-1000
  learning_objectives:
    - "Describe simulated sensor types and their configurations"
    - "Explain noise models and their impact on perception"
    - "Understand sim-to-real gap for perception pipelines"
  key_concepts:
    - camera-simulation
    - lidar-simulation
    - imu-simulation
    - force-torque-simulation
    - noise-models
    - sim-to-real-perception
  diagrams: 1-2
  code_snippets: 1
  prerequisites_from_module1:
    - rclpy publishers/subscribers (Chapter 3)
```

## RAG Chunk Definitions

### Chapter 1 Chunks

| Slug | Content Scope | Keywords |
|------|--------------|----------|
| `digital-twin-definition` | What is a digital twin, core components | digital twin, virtual replica, synchronization |
| `simulation-benefits` | Safe testing, rapid iteration, cost reduction | simulation, testing, iteration, development |
| `fidelity-levels` | Low/medium/high fidelity trade-offs | fidelity, accuracy, performance, trade-off |
| `simulation-ecosystem` | Physics engines, renderers, sensors overview | ecosystem, physics, rendering, sensors |

### Chapter 2 Chunks

| Slug | Content Scope | Keywords |
|------|--------------|----------|
| `gazebo-architecture` | Gazebo components and structure | Gazebo, architecture, server, client |
| `physics-engines` | ODE, Bullet, DART, Simbody comparison | physics engine, ODE, Bullet, DART, Simbody |
| `world-modeling` | Gravity, friction, contact dynamics | world, gravity, friction, contact, dynamics |
| `urdf-sdf-conversion` | URDF to SDF pipeline, model spawning | URDF, SDF, conversion, spawn |
| `ros2-control-integration` | gz-ros2-control patterns | ros2_control, Gazebo, integration |

### Chapter 3 Chunks

| Slug | Content Scope | Keywords |
|------|--------------|----------|
| `unity-robotics-visualization` | Unity for robotics, rendering capabilities | Unity, visualization, rendering, robotics |
| `rendering-vs-physics` | Trade-offs between visual and physical accuracy | rendering, physics, accuracy, trade-off |
| `human-robot-interaction` | HRI scenarios in Unity | HRI, human-robot interaction, Unity |
| `ros-tcp-connector` | Unity-ROS bridge concepts | ROS-TCP-Connector, bridge, communication |
| `gazebo-unity-comparison` | When to use each tool | Gazebo, Unity, comparison, selection |

### Chapter 4 Chunks

| Slug | Content Scope | Keywords |
|------|--------------|----------|
| `camera-simulation` | Simulated cameras, RGB, depth | camera, RGB, depth, simulation |
| `lidar-simulation` | LiDAR sensors, point clouds | LiDAR, point cloud, laser, simulation |
| `imu-simulation` | IMU sensors, accelerometer, gyroscope | IMU, accelerometer, gyroscope, simulation |
| `force-torque-simulation` | Force-torque sensors | force, torque, sensor, contact |
| `noise-models` | Gaussian noise, calibration concepts | noise, Gaussian, calibration, error |
| `sim-to-real-perception` | Gap mitigation for perception | sim-to-real, perception, domain randomization |

## Cross-Module Relationships

```
Module 1 Concepts → Module 2 Usage:
┌────────────────────────────────────┐
│ Module 1, Chapter 4: URDF          │
│ - Links, joints, transmissions     │
│ - Robot description format         │
└──────────────┬─────────────────────┘
               │ Referenced by
               ▼
┌────────────────────────────────────┐
│ Module 2, Chapter 2: Gazebo        │
│ - URDF-to-SDF conversion           │
│ - Model spawning in simulation     │
└────────────────────────────────────┘

┌────────────────────────────────────┐
│ Module 1, Chapter 2: Core Primitives│
│ - Topics, Services, Actions        │
│ - Message passing                  │
└──────────────┬─────────────────────┘
               │ Referenced by
               ▼
┌────────────────────────────────────┐
│ Module 2, Chapter 2: Gazebo        │
│ - gz-ros2-control integration      │
│ - Sensor data publishing           │
└────────────────────────────────────┘

┌────────────────────────────────────┐
│ Module 1, Chapter 3: rclpy         │
│ - Publishers, Subscribers          │
│ - Callbacks and executors          │
└──────────────┬─────────────────────┘
               │ Referenced by
               ▼
┌────────────────────────────────────┐
│ Module 2, Chapter 4: Sensors       │
│ - Sensor data subscription         │
│ - Processing pipelines             │
└────────────────────────────────────┘
```

## Frontmatter Schema

```yaml
# Required fields for all chapters
---
id: string              # URL slug (e.g., "digital-twins")
title: string           # Full title with quotes
sidebar_label: string   # Short sidebar label
sidebar_position: int   # 1-4 within module
description: string     # SEO description (150-160 chars)
keywords: string[]      # 5-10 keywords for RAG/SEO
learning_objectives:    # 2-4 objectives
  - string
---
```

## Content Structure Contract

Each chapter MUST include:

1. **Frontmatter** (per schema above)
2. **Title** (H1, matches frontmatter title)
3. **Prerequisites Note** (if references Module 1)
4. **Main Content Sections** (H2 headings)
5. **RAG Chunk Markers** (inline comments)
6. **Diagram Descriptions** (:::info admonition)
7. **Code Snippets** (with disclaimer comments, if applicable)
8. **Key Takeaways** (H2, 4-6 bullet points)
9. **Next Chapter Link** (except Chapter 4)

## Validation Rules

| Rule | Check | Pass Criteria |
|------|-------|---------------|
| Word count per chapter | `wc -w` | 625-1000 words |
| Total word count | Sum of chapters | 2,500-4,000 words |
| Frontmatter complete | YAML parse | All required fields present |
| Sidebar position | Frontmatter check | Sequential 1-4 |
| Learning objectives | Count | 2-4 per chapter |
| Key takeaways | Section check | Present with 4-6 bullets |
| RAG chunks | Grep `<!-- chunk:` | At least 2 per chapter |
| Diagrams | Grep `:::info Diagram` | At least 1 per chapter |
| Code disclaimers | Grep `NOTE:` | All snippets have disclaimer |
| Cross-links | Link validation | No broken references |
