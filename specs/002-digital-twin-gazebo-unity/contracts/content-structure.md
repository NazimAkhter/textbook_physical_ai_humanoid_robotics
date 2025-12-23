# Content Structure Contract: Module 2 – The Digital Twin

**Date**: 2024-12-14
**Feature**: `002-digital-twin-gazebo-unity`
**Purpose**: Define structural requirements for chapter files

## Directory Structure

```
frontend/docs/module-02-digital-twin/
├── _category_.json           # REQUIRED: Sidebar configuration
├── 01-digital-twins.md       # REQUIRED: Chapter 1
├── 02-gazebo-physics.md      # REQUIRED: Chapter 2
├── 03-unity-visualization.md # REQUIRED: Chapter 3
└── 04-sensor-simulation.md   # REQUIRED: Chapter 4
```

## _category_.json Contract

```json
{
  "label": "Module 2: Digital Twins",
  "position": 2,
  "collapsed": false,
  "link": {
    "type": "generated-index",
    "description": "Learn how digital twins enable safe robotics development through physics simulation, visualization, and sensor modeling."
  }
}
```

## Chapter File Contract

### Frontmatter (REQUIRED)

```yaml
---
id: <slug>                    # URL-safe identifier
title: "<Full Title>"         # H1 title
sidebar_label: "<Short>"      # Sidebar display
sidebar_position: <1-4>       # Order within module
description: "<150-160 chars>" # SEO/preview text
keywords: ["kw1", "kw2", ...]  # 5-10 keywords
learning_objectives:
  - "Objective 1"
  - "Objective 2"
  - "Objective 3"
---
```

### Body Structure (REQUIRED)

```markdown
# [Title from frontmatter]

:::note Prerequisites
This chapter builds on [concept](../module-01-ros2-nervous-system/chapter.md).
[Include only if referencing Module 1]
:::

<!-- chunk:concept-slug -->

## Section Heading

[Content...]

<!-- /chunk -->

[Repeat for each major section]

:::info Diagram: [Diagram Title]

**Type**: [architecture/comparison/data-flow/hierarchy]

**Description**: [What the diagram shows]

**Key Elements**:
- Element 1
- Element 2
- Element 3

:::

## Key Takeaways

- **Point 1**: [Summary statement]
- **Point 2**: [Summary statement]
- **Point 3**: [Summary statement]
- **Point 4**: [Summary statement]

---

*Next: [Next Chapter Title](./next-chapter.md) — [Brief description]*
```

### Code Snippet Contract (IF APPLICABLE)

```markdown
```language title="Descriptive Title"
<!-- NOTE: This is an illustrative example, not production code -->
[Code content]
```
```

## Chapter-Specific Contracts

### Chapter 1: Digital Twins in Robotics

**File**: `01-digital-twins.md`

**Required Sections**:
- What is a Digital Twin?
- Benefits of Simulation-Driven Development
- Simulation Fidelity Levels
- The Simulation Ecosystem (overview)
- Key Takeaways

**RAG Chunks**:
- `digital-twin-definition`
- `simulation-benefits`
- `fidelity-levels`
- `simulation-ecosystem`

**Diagrams**: 1-2
- Recommended: Digital Twin Architecture Overview

**Code Snippets**: 0

---

### Chapter 2: Physics Simulation with Gazebo

**File**: `02-gazebo-physics.md`

**Required Sections**:
- Gazebo Architecture
- Physics Engines (ODE, Bullet, DART, Simbody)
- World Modeling (gravity, friction, contacts)
- URDF to SDF Conversion
- ROS 2 Integration (gz-ros2-control)
- Key Takeaways

**RAG Chunks**:
- `gazebo-architecture`
- `physics-engines`
- `world-modeling`
- `urdf-sdf-conversion`
- `ros2-control-integration`

**Diagrams**: 2
- Gazebo Architecture
- URDF-to-SDF Conversion Flow

**Code Snippets**: 1-2
- SDF world fragment (XML)
- Physics parameter configuration (XML)

**Cross-References**:
- Module 1, Chapter 4 (URDF structure)
- Module 1, Chapter 2 (ROS 2 topics/services)

---

### Chapter 3: Visualization and Interaction with Unity

**File**: `03-unity-visualization.md`

**Required Sections**:
- Unity for Robotics Visualization
- Rendering vs. Physics Accuracy
- Human-Robot Interaction Scenarios
- Unity-ROS Integration (ROS-TCP-Connector)
- Gazebo vs. Unity Comparison
- Key Takeaways

**RAG Chunks**:
- `unity-robotics-visualization`
- `rendering-vs-physics`
- `human-robot-interaction`
- `ros-tcp-connector`
- `gazebo-unity-comparison`

**Diagrams**: 1-2
- Unity-ROS Integration Architecture
- Gazebo vs Unity Comparison Table (can be markdown table)

**Code Snippets**: 0-1
- Conceptual Unity-ROS bridge (if included)

---

### Chapter 4: Sensor Simulation for Perception

**File**: `04-sensor-simulation.md`

**Required Sections**:
- Simulated Sensors Overview
- Camera Simulation (RGB, depth)
- LiDAR Simulation
- IMU Simulation
- Force-Torque Sensors
- Noise Models and Calibration
- Sim-to-Real Gap for Perception
- Key Takeaways

**RAG Chunks**:
- `camera-simulation`
- `lidar-simulation`
- `imu-simulation`
- `force-torque-simulation`
- `noise-models`
- `sim-to-real-perception`

**Diagrams**: 1-2
- Sensor Data Flow in Simulation
- Noise Model Illustration

**Code Snippets**: 1
- Sensor noise configuration concept (YAML or XML)

**Cross-References**:
- Module 1, Chapter 3 (rclpy publishers/subscribers)

---

## Validation Checklist

### Per-Chapter Checks

- [ ] Frontmatter complete with all required fields
- [ ] Title matches frontmatter title
- [ ] sidebar_position matches chapter number
- [ ] Prerequisites note included (if cross-referencing Module 1)
- [ ] All required sections present
- [ ] RAG chunk markers present for each section
- [ ] At least 1 diagram description
- [ ] Code snippets have disclaimer (if present)
- [ ] Key Takeaways section with 4-6 bullets
- [ ] Next chapter link (except Chapter 4)
- [ ] Word count 625-1000 words

### Module-Level Checks

- [ ] _category_.json present and valid
- [ ] All 4 chapter files present
- [ ] sidebar_position values 1-4 (no gaps)
- [ ] Total word count 2,500-4,000
- [ ] No broken cross-references
- [ ] Docusaurus build succeeds
- [ ] Sidebar shows correct order
