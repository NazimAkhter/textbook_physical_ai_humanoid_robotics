# Content Structure Contract: Module 1

**Feature Branch**: `001-ros2-nervous-system`
**Date**: 2025-12-14
**Version**: 1.0.0

## Overview

This contract defines the structural requirements for Module 1 content files. All content must conform to these specifications to ensure consistency and RAG compatibility.

## File System Contract

### Directory Structure

```
docs/
└── module-01-ros2-nervous-system/
    ├── _category_.json
    ├── index.md
    ├── 01-ros2-overview.md
    ├── 02-core-primitives.md
    ├── 03-rclpy-integration.md
    └── 04-urdf-humanoids.md
```

### File Naming Convention

| Pattern | Description | Example |
|---------|-------------|---------|
| `_category_.json` | Sidebar category configuration | Required at module root |
| `index.md` | Module overview page | Optional, recommended |
| `{NN}-{slug}.md` | Chapter content file | `01-ros2-overview.md` |

## Category Configuration Contract

**File**: `_category_.json`

```json
{
  "label": "Module 1: ROS 2 Nervous System",
  "position": 1,
  "collapsed": false,
  "link": {
    "type": "generated-index",
    "description": "Learn how ROS 2 serves as the nervous system for humanoid robots."
  }
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `label` | string | Yes | Sidebar display name |
| `position` | integer | Yes | Module order (1-5) |
| `collapsed` | boolean | No | Default collapse state |
| `link.type` | string | No | Index page type |
| `link.description` | string | No | Index page description |

## Frontmatter Contract

Every chapter Markdown file MUST include this frontmatter structure:

```yaml
---
id: string           # Required: unique identifier (e.g., "ros2-overview")
title: string        # Required: full page title
sidebar_label: string # Required: short sidebar label (max 25 chars)
sidebar_position: integer # Required: navigation order (1-4)
description: string  # Required: SEO/RAG description (max 160 chars)
keywords: [string]   # Required: 3-7 discovery keywords
learning_objectives: [string] # Required: 2-4 chapter objectives
---
```

### Example Frontmatter

```yaml
---
id: ros2-overview
title: "ROS 2 as the Robotic Nervous System"
sidebar_label: "ROS 2 Overview"
sidebar_position: 1
description: "Understand ROS 2's role as message-passing middleware for humanoid robots, comparing it to biological nervous systems."
keywords: ["ROS 2", "middleware", "DDS", "robot communication", "nervous system"]
learning_objectives:
  - "Define ROS 2 and explain its role as middleware"
  - "Compare ROS 2 communication to biological nervous system functions"
  - "Describe the purpose of DDS in real-time robot communication"
---
```

## Content Section Contract

Each chapter MUST include these sections in order:

| Section | Required | Description |
|---------|----------|-------------|
| Introduction | Yes | 1-2 paragraphs setting context |
| Main Content | Yes | Core educational material with headings |
| Code Examples | Conditional | At least 1 per chapter (except Ch.1) |
| Diagrams | Yes | At least 1 diagram description per chapter |
| Key Takeaways | Yes | 3-5 bullet summary (FR-009) |
| Next Steps | No | Link to next chapter (optional) |

### Code Block Contract

All code blocks MUST follow this format:

````markdown
```python title="Descriptive Title"
# NOTE: This is an illustrative example, not production code
code_here
```
````

| Attribute | Required | Description |
|-----------|----------|-------------|
| Language identifier | Yes | `python`, `xml`, or `bash` |
| `title` | Yes | Descriptive title for the snippet |
| Disclaimer comment | Yes | First line indicates non-executable |

### Diagram Description Contract

Diagrams are described textually using this format:

```markdown
:::info Diagram: [Caption]

**Type**: [architecture | data-flow | comparison | hierarchy]

**Description**: [What the diagram shows]

**Key Elements**:
- Element 1: [Description]
- Element 2: [Description]
- Element 3: [Description]

:::
```

## RAG Chunking Contract

Content may include semantic chunking markers for RAG processing:

```markdown
<!-- chunk:concept-slug -->
Content that forms a single semantic unit...
<!-- /chunk -->
```

| Marker | Description |
|--------|-------------|
| `<!-- chunk:slug -->` | Start of semantic chunk |
| `<!-- /chunk -->` | End of semantic chunk |

**Chunking Guidelines**:
- Maximum ~500 tokens per chunk
- One primary concept per chunk
- Include surrounding context for comprehension
- Avoid splitting code blocks across chunks

## Validation Checklist

- [ ] All 4 chapter files exist with correct naming
- [ ] `_category_.json` present with valid JSON
- [ ] All frontmatter fields populated
- [ ] `sidebar_position` matches chapter order (1-4)
- [ ] Each chapter has Key Takeaways section
- [ ] Code blocks have language identifier and title
- [ ] Diagram descriptions use :::info admonition
- [ ] No content exceeds module scope boundaries
