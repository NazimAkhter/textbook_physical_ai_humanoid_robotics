# Quickstart: Module 2 – The Digital Twin (Gazebo & Unity)

**Date**: 2024-12-14
**Feature**: `002-digital-twin-gazebo-unity`
**Purpose**: Implementation guide for content authors

## Prerequisites

- Module 1 content implemented in `frontend/docs/module-01-ros2-nervous-system/`
- Docusaurus project builds successfully
- Familiarity with Markdown and frontmatter conventions

## Quick Start Steps

### 1. Create Module Directory

```bash
mkdir -p frontend/docs/module-02-digital-twin
```

### 2. Add Sidebar Configuration

Create `frontend/docs/module-02-digital-twin/_category_.json`:

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

### 3. Create Chapter Stubs

Create each chapter file with frontmatter:

**01-digital-twins.md**:
```yaml
---
id: digital-twins
title: "Digital Twins in Robotics"
sidebar_label: "Digital Twins"
sidebar_position: 1
description: "Understand digital twins as virtual representations enabling safe robotics development and testing."
keywords: ["digital twin", "simulation", "robotics", "virtual testing", "fidelity"]
learning_objectives:
  - "Define digital twin and explain its role in robotics"
  - "Identify benefits of simulation-driven development"
  - "Compare simulation fidelity levels"
---

# Digital Twins in Robotics

[Content to be added]
```

**02-gazebo-physics.md**:
```yaml
---
id: gazebo-physics
title: "Physics Simulation with Gazebo"
sidebar_label: "Gazebo Physics"
sidebar_position: 2
description: "Learn Gazebo architecture, physics engines, and ROS 2 integration for humanoid robot simulation."
keywords: ["Gazebo", "physics simulation", "ODE", "Bullet", "DART", "ROS 2", "gz-ros2-control"]
learning_objectives:
  - "Describe Gazebo architecture and physics engine options"
  - "Explain world modeling concepts"
  - "Understand URDF-to-SDF conversion"
---

# Physics Simulation with Gazebo

[Content to be added]
```

**03-unity-visualization.md**:
```yaml
---
id: unity-visualization
title: "Visualization and Interaction with Unity"
sidebar_label: "Unity Visualization"
sidebar_position: 3
description: "Explore Unity for robotics visualization, human-robot interaction, and ROS integration."
keywords: ["Unity", "visualization", "HRI", "ROS-TCP-Connector", "rendering", "robotics"]
learning_objectives:
  - "Explain when to use Unity for robotics"
  - "Understand Unity-ROS integration concepts"
  - "Compare Gazebo and Unity use cases"
---

# Visualization and Interaction with Unity

[Content to be added]
```

**04-sensor-simulation.md**:
```yaml
---
id: sensor-simulation
title: "Sensor Simulation for Perception"
sidebar_label: "Sensor Simulation"
sidebar_position: 4
description: "Learn sensor simulation concepts including cameras, LiDAR, IMU, and noise modeling for perception pipelines."
keywords: ["sensor simulation", "camera", "LiDAR", "IMU", "noise model", "sim-to-real", "perception"]
learning_objectives:
  - "Describe simulated sensor types and configurations"
  - "Explain noise models and calibration"
  - "Understand sim-to-real gap for perception"
---

# Sensor Simulation for Perception

[Content to be added]
```

### 4. Verify Build

```bash
cd frontend
npm run build
```

Verify:
- No build errors
- Module 2 appears in sidebar after Module 1
- All 4 chapters visible in correct order

### 5. Content Development Workflow

For each chapter:

1. **Add RAG chunk markers** around major sections:
   ```markdown
   <!-- chunk:concept-name -->
   ## Section Title
   [Content]
   <!-- /chunk -->
   ```

2. **Add diagram descriptions**:
   ```markdown
   :::info Diagram: Title
   **Type**: architecture
   **Description**: What it shows
   **Key Elements**:
   - Element 1
   - Element 2
   :::
   ```

3. **Add code snippets** (if applicable):
   ```markdown
   ```xml title="Example Title"
   <!-- NOTE: This is an illustrative example, not production code -->
   [Code]
   ```
   ```

4. **Add Key Takeaways**:
   ```markdown
   ## Key Takeaways
   - **Point 1**: Summary
   - **Point 2**: Summary
   ```

5. **Add navigation link** (except Chapter 4):
   ```markdown
   ---
   *Next: [Next Chapter](./next-chapter.md) — Brief description*
   ```

### 6. Validation

After completing content:

```bash
# Word count check
wc -w frontend/docs/module-02-digital-twin/*.md

# Build verification
cd frontend && npm run build

# Check for required elements
grep -l "## Key Takeaways" frontend/docs/module-02-digital-twin/*.md
grep -l "<!-- chunk:" frontend/docs/module-02-digital-twin/*.md
grep -l ":::info Diagram" frontend/docs/module-02-digital-twin/*.md
```

## Content Guidelines

### Word Targets

| Chapter | Target | Range |
|---------|--------|-------|
| Chapter 1 | ~750 words | 625-1000 |
| Chapter 2 | ~850 words | 625-1000 |
| Chapter 3 | ~750 words | 625-1000 |
| Chapter 4 | ~800 words | 625-1000 |
| **Total** | ~3150 words | 2500-4000 |

### Cross-Reference Format

When referencing Module 1:
```markdown
:::note Prerequisites
This chapter builds on [URDF concepts from Module 1](../module-01-ros2-nervous-system/04-urdf-humanoids.md).
:::
```

### Code Snippet Rules

1. Always include disclaimer comment
2. Keep snippets under 20 lines
3. Use appropriate syntax highlighting
4. Focus on configuration/structure, not implementation

## Troubleshooting

### Sidebar Not Showing Module 2

- Check `_category_.json` exists and is valid JSON
- Verify `position: 2` is set
- Run `npm run build` to regenerate

### Build Errors

- Check frontmatter YAML syntax (quotes, indentation)
- Verify all markdown links are valid
- Check for unclosed code blocks

### Word Count Issues

- Use `wc -w` to count actual words
- Frontmatter and code blocks count toward total
- Adjust content density if needed

## Reference Files

- **Spec**: `specs/002-digital-twin-gazebo-unity/spec.md`
- **Plan**: `specs/002-digital-twin-gazebo-unity/plan.md`
- **Data Model**: `specs/002-digital-twin-gazebo-unity/data-model.md`
- **Contract**: `specs/002-digital-twin-gazebo-unity/contracts/content-structure.md`
- **Module 1 Example**: `frontend/docs/module-01-ros2-nervous-system/`
