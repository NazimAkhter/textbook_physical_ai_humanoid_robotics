# Quickstart: Writing Module 3 Chapters

**Date**: 2024-12-16
**Purpose**: Guidelines for writing Module 3 content consistently

## Chapter Template

```markdown
---
id: [slug]
title: "[Full Title]"
sidebar_label: "[Short Label]"
sidebar_position: [1-4]
description: "[SEO-friendly description, 150-160 chars]"
keywords: ["keyword1", "keyword2", "keyword3", ...]
learning_objectives:
  - "[Testable objective 1]"
  - "[Testable objective 2]"
  - "[Testable objective 3]"
---

# [Title]

[Optional: Prerequisites callout if chapter has specific dependencies]

<!-- chunk:introduction -->

## [Opening Section]

[Hook paragraph connecting to previous modules or real-world motivation]
[Core concept introduction]

<!-- /chunk -->

<!-- chunk:concept-1 -->

## [Main Concept Section]

[Technical explanation with depth]
[Tables or comparisons where helpful]

:::info Diagram: [Diagram Title]

**Type**: [architecture/flow/comparison]

**Description**: [What the diagram shows]

**Key Elements**:
- [Element 1]
- [Element 2]
- [Element 3]

:::

<!-- /chunk -->

<!-- chunk:concept-2 -->

## [Secondary Concept Section]

[Additional technical content]

```python
# Conceptual code snippet
# NOT a full tutorial - illustrative only
```

<!-- /chunk -->

## Key Takeaways

- [Bullet 1: Main concept summary]
- [Bullet 2: Key insight]
- [Bullet 3: Connection to broader system]
- [Optional Bullet 4-5]

## Next Steps

[Transition to next chapter or module]

[Optional: External resources for readers wanting tutorials]
```

## Writing Guidelines

### Tone
- **Technical precision**: Use correct terminology
- **Non-marketing**: Avoid superlatives like "revolutionary", "cutting-edge", "game-changing"
- **Neutral comparisons**: When mentioning alternatives, compare factually
- **Educational focus**: Explain concepts, don't sell products

### Structure
- **Word count**: 1500-2500 words per chapter (500-4000 allowed)
- **Sections**: 3-5 main sections per chapter
- **Code snippets**: Conceptual only, syntactically valid, well-commented
- **Diagrams**: At least 1 per chapter, described in markdown info blocks

### Chunk Comments
- Every major section should be wrapped in chunk comments
- Format: `<!-- chunk:descriptive-name -->` ... `<!-- /chunk -->`
- Chunks enable RAG retrieval - each should be self-contained

### Cross-References
- Reference Module 1-2 concepts by chapter name
- Example: "As discussed in Module 1's Core Primitives chapter..."
- Don't repeat content - link back

### Frontmatter Requirements
| Field | Required | Example |
|-------|----------|---------|
| id | Yes | `ai-robot-brain` |
| title | Yes | `"The AI-Robot Brain: Isaac Architecture"` |
| sidebar_label | Yes | `"Isaac Architecture"` |
| sidebar_position | Yes | `1` |
| description | Yes | `"Learn how NVIDIA Isaac connects perception, simulation, and navigation in humanoid robots."` |
| keywords | Yes | `["nvidia isaac", "robotics ai", "perception"]` |
| learning_objectives | Yes | 3 testable objectives |

## Quality Checklist

Before submitting each chapter:

- [ ] Word count within 500-4000 range
- [ ] All frontmatter fields complete
- [ ] Learning objectives are testable (reader can demonstrate)
- [ ] Chunk comments wrap all major sections
- [ ] At least one diagram described
- [ ] Key takeaways section (3-5 bullets)
- [ ] Next steps section connects to next chapter
- [ ] No marketing language
- [ ] Code snippets are syntactically valid
- [ ] Cross-references to Module 1-2 are accurate
- [ ] Technical terms defined on first use or in glossary reference

## File Naming Convention

```
frontend/docs/module-03-nvidia-isaac/
├── _category_.json           # Sidebar configuration
├── 01-ai-robot-brain.md      # Chapter 1
├── 02-isaac-sim.md           # Chapter 2
├── 03-isaac-ros.md           # Chapter 3
└── 04-nav2-navigation.md     # Chapter 4
```

## _category_.json Template

```json
{
  "label": "Module 3: AI-Robot Brain",
  "position": 3,
  "collapsed": false,
  "link": {
    "type": "generated-index",
    "description": "Learn how NVIDIA Isaac provides the AI brain for humanoid robots, connecting photorealistic simulation, accelerated perception, and autonomous navigation."
  }
}
```

## Example Learning Objectives

### Good (Testable)
- "Explain how Isaac connects perception, localization, and navigation in a data flow diagram"
- "Describe three techniques for closing the sim-to-real gap"
- "Identify which Nav2 components handle global vs local planning"

### Bad (Not Testable)
- "Understand Isaac architecture" (too vague)
- "Appreciate the power of GPU acceleration" (not measurable)
- "Know about Visual SLAM" (passive knowledge)

## Diagram Description Format

Use Docusaurus admonitions for diagram placeholders:

```markdown
:::info Diagram: Isaac Platform Architecture

**Type**: architecture

**Description**: Layered diagram showing how Isaac Sim, Isaac ROS, and Nav2 connect from simulation through perception to navigation control.

**Key Elements**:
- Isaac Sim layer: Omniverse, USD scenes, synthetic data output
- Isaac ROS layer: cuVSLAM, depth estimation, object detection nodes
- Nav2 layer: behavior tree, global planner, local planner, costmap
- ROS 2 Control layer: joint controllers (reference Module 1)
- Arrows showing data flow between layers

:::
```

## Code Snippet Guidelines

1. **Language**: Python for ROS 2/Isaac, YAML for configs, XML for behavior trees
2. **Comments**: Explain what each section does
3. **Validity**: Code should be syntactically correct
4. **Scope**: Illustrative only - readers should not expect to run it
5. **Attribution**: Note if adapted from official documentation

Example:
```python
# Conceptual Isaac ROS launch configuration
# Shows how perception nodes integrate with standard ROS 2
# NOT executable without proper Isaac ROS installation

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node

def generate_launch_description():
    # Visual SLAM node with GPU acceleration
    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # cuVSLAM for visual-inertial odometry
            # Outputs: /visual_slam/pose, /visual_slam/odom
        ]
    )
    return LaunchDescription([vslam_container])
```
