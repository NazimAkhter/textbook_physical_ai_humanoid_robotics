# Research: Module 2 – The Digital Twin (Gazebo & Unity)

**Date**: 2024-12-14
**Feature**: `002-digital-twin-gazebo-unity`
**Purpose**: Document research decisions, authoritative references, and technical approach

## Research Approach

Research-concurrent writing with APA citation style per constitution guidelines. Focus on authoritative technical sources for simulation concepts.

## Key Decisions

### 1. Docusaurus Configuration

**Decision**: Reuse Module 1 configuration pattern

**Rationale**: Module 1 successfully implemented with:
- Auto-generated sidebar via `_category_.json`
- Frontmatter-based ordering (`sidebar_position`)
- Docs plugin with Classic theme

No changes required for Module 2 integration.

### 2. Module/Chapter Structuring Strategy

**Decision**: Single module directory with numbered chapter files

**Pattern**:
```
frontend/docs/module-02-digital-twin/
├── _category_.json           # Sidebar: label, position, collapsed
├── 01-digital-twins.md       # sidebar_position: 1
├── 02-gazebo-physics.md      # sidebar_position: 2
├── 03-unity-visualization.md # sidebar_position: 3
└── 04-sensor-simulation.md   # sidebar_position: 4
```

**Rationale**: Mirrors Module 1 structure for consistency. Predictable URLs enable RAG chunking.

### 3. Frontmatter Standards

**Decision**: Extended frontmatter for RAG and SEO

**Standard Fields**:
```yaml
---
id: <slug>
title: "<Full Chapter Title>"
sidebar_label: "<Short Label>"
sidebar_position: <1-4>
description: "<SEO description, 150-160 chars>"
keywords: ["keyword1", "keyword2", ...]
learning_objectives:
  - "Objective 1"
  - "Objective 2"
  - "Objective 3"
---
```

**Rationale**: Consistent with Module 1. Keywords and objectives support RAG retrieval and learning validation.

### 4. Navigation and Sidebar Approach

**Decision**: Auto-generated sidebar with explicit module ordering

**`_category_.json` for Module 2**:
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

**Sidebar Hierarchy**:
- Module 1: ROS 2 Nervous System (position: 1)
- Module 2: Digital Twins (position: 2) ← This module
- Module 3-5: Future modules

### 5. RAG Content Reuse Strategy

**Decision**: Inline chunking markers with semantic slugs

**Chunking Pattern**:
```markdown
<!-- chunk:digital-twin-definition -->
## What is a Digital Twin?
[Content...]
<!-- /chunk -->
```

**Slug Conventions**:
- Chapter 1: `digital-twin-*` (definition, benefits, fidelity-levels, ecosystem)
- Chapter 2: `gazebo-*` (architecture, physics-engines, world-modeling, urdf-sdf, ros2-control)
- Chapter 3: `unity-*` (visualization, hri, ros-bridge, comparison)
- Chapter 4: `sensor-*` (cameras, lidar, imu, force-torque, noise-models, sim2real)

**Rationale**: Semantic slugs enable targeted retrieval. Consistent pattern with Module 1.

### 6. Cross-Module Linking Strategy

**Decision**: Explicit prerequisite notes and inline references

**Pattern**:
```markdown
:::note Prerequisites
This chapter builds on [URDF concepts from Module 1](../module-01-ros2-nervous-system/04-urdf-humanoids.md). Understanding links, joints, and robot descriptions is essential.
:::
```

**Cross-Reference Points**:
- Chapter 2 (Gazebo) → Module 1 Chapter 4 (URDF)
- Chapter 2 (Gazebo) → Module 1 Chapter 2 (ROS 2 Topics/Services)
- Chapter 4 (Sensors) → Module 1 Chapter 3 (rclpy publishers/subscribers)

## Authoritative References

### Digital Twins

| Topic | Source | Usage |
|-------|--------|-------|
| Digital twin concept | Grieves, M. (2014). Digital twin: Manufacturing excellence through virtual factory replication | Definition and historical context |
| NASA digital twin | Glaessgen, E., & Stargel, D. (2012). The digital twin paradigm for future NASA and US Air Force vehicles | Aerospace origins |
| Robotics application | Barricelli, B. R., et al. (2019). A survey on digital twin: Definitions, characteristics, applications, and design implications | Survey of applications |

### Gazebo Simulation

| Topic | Source | Usage |
|-------|--------|-------|
| Gazebo architecture | Open Robotics. (2024). Gazebo documentation. https://gazebosim.org/docs | Official architecture reference |
| Physics engines | Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo | Physics engine comparison |
| ROS 2 integration | ros2_control documentation. https://control.ros.org/ | gz-ros2-control patterns |

### Unity Robotics

| Topic | Source | Usage |
|-------|--------|-------|
| Unity Robotics Hub | Unity Technologies. (2024). Unity Robotics Hub. https://github.com/Unity-Technologies/Unity-Robotics-Hub | Official documentation |
| ROS-TCP-Connector | Unity Technologies. (2024). ROS-TCP-Connector. | ROS-Unity bridge |
| Simulation comparison | Various industry analyses | Gazebo vs Unity trade-offs |

### Sensor Simulation

| Topic | Source | Usage |
|-------|--------|-------|
| Sensor noise models | Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic robotics | Noise modeling fundamentals |
| LiDAR simulation | Gazebo sensor plugins documentation | LiDAR configuration |
| Synthetic data | Tremblay, J., et al. (2018). Training deep networks with synthetic data | Domain randomization |

## Content Guidelines

### Tone and Style

- **Conceptual**: Explain "what" and "why" before "how"
- **Systems-oriented**: Focus on architecture and data flow
- **Comparative**: Help readers understand tool selection criteria
- **Humanoid-focused**: Use humanoid robot examples consistently

### Code Snippet Rules

1. All snippets MUST include disclaimer comment
2. Use syntax highlighting for appropriate language (xml, python, yaml)
3. Keep snippets short (10-20 lines max)
4. Focus on configuration/structure, not implementation

**Example Disclaimer**:
```xml
<!-- NOTE: This is an illustrative example, not production code -->
```

### Diagram Description Format

```markdown
:::info Diagram: [Title]

**Type**: [architecture/comparison/data-flow/hierarchy]

**Description**: [What the diagram shows]

**Key Elements**:
- Element 1
- Element 2
- Element 3

:::
```

## Terminology Glossary

| Term | Definition | Notes |
|------|------------|-------|
| Digital Twin | Virtual representation synchronized with physical robot | Core concept |
| Physics Engine | Component computing dynamics (ODE, Bullet, DART) | Gazebo chapter |
| SDF | Simulation Description Format | Gazebo's native format |
| World Model | Environment description with physics properties | Gazebo chapter |
| HRI | Human-Robot Interaction | Unity chapter |
| Sim-to-Real Gap | Discrepancy between simulation and reality | Cross-chapter concept |
| Domain Randomization | Varying simulation parameters for robustness | Sensor chapter |
| Noise Model | Statistical model of sensor measurement errors | Sensor chapter |

## Out-of-Scope Reminders

Per spec, explicitly avoid:
- Installation guides for Gazebo or Unity
- Step-by-step simulation tutorials
- Sim-to-real deployment procedures
- Machine learning / VLA content
- Production game development techniques
- Real-time rendering optimization

## Research Status

| Category | Status | Notes |
|----------|--------|-------|
| Digital twin concepts | ✅ Complete | Academic and industry sources identified |
| Gazebo architecture | ✅ Complete | Official documentation referenced |
| Unity Robotics | ✅ Complete | Unity-Technologies repos referenced |
| Sensor simulation | ✅ Complete | Probabilistic robotics fundamentals |
| Cross-module links | ✅ Complete | Module 1 integration points identified |
