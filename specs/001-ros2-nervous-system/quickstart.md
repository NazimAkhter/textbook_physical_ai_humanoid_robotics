# Quickstart: Module 1 – The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-nervous-system`
**Date**: 2025-12-14

## Prerequisites

Before implementing Module 1 content, ensure:

1. **Docusaurus Project Initialized**
   - Docusaurus 3.x installed and configured
   - Classic theme with docs plugin enabled
   - Project builds successfully (`npm run build`)

2. **Directory Structure Created**
   - `docs/module-01-ros2-nervous-system/` directory exists
   - `_category_.json` configured for sidebar

3. **Constitution Compliance Verified**
   - Content follows Spec-Kit Plus workflow
   - Technical standards aligned with project constitution

## Quick Implementation Steps

### Step 1: Create Module Directory

```bash
mkdir -p docs/module-01-ros2-nervous-system
```

### Step 2: Add Category Configuration

Create `docs/module-01-ros2-nervous-system/_category_.json`:

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

### Step 3: Create Chapter Files

Create the following files with frontmatter:

| File | Sidebar Position |
|------|------------------|
| `01-ros2-overview.md` | 1 |
| `02-core-primitives.md` | 2 |
| `03-rclpy-integration.md` | 3 |
| `04-urdf-humanoids.md` | 4 |

### Step 4: Frontmatter Template

Use this template for each chapter:

```yaml
---
id: [chapter-slug]
title: "[Full Chapter Title]"
sidebar_label: "[Short Label]"
sidebar_position: [1-4]
description: "[SEO description, max 160 chars]"
keywords: ["keyword1", "keyword2", "keyword3"]
learning_objectives:
  - "Objective 1"
  - "Objective 2"
---

# [Chapter Title]

[Introduction paragraph]

## [Section Heading]

[Content]

## Key Takeaways

- Takeaway 1
- Takeaway 2
- Takeaway 3
```

### Step 5: Validate Structure

Run local development server:

```bash
npm run start
```

Verify:
- [ ] Module appears in sidebar at position 1
- [ ] All 4 chapters display in correct order
- [ ] Navigation links work between chapters
- [ ] No build errors or warnings

## Content Guidelines Summary

| Guideline | Requirement |
|-----------|-------------|
| Total word count | 2,500–4,000 words |
| Chapters | Exactly 4, in fixed order |
| Code snippets | Minimal, illustrative, non-executable |
| Diagrams | At least 1 per chapter (textual description) |
| Key Takeaways | Required for each chapter |
| Tone | Educational, systems-oriented |

## Validation Commands

```bash
# Build check
npm run build

# Word count (approximate)
find docs/module-01-ros2-nervous-system -name "*.md" -exec wc -w {} +

# Link check
npm run serve && npx broken-link-checker http://localhost:3000/docs/module-01-ros2-nervous-system
```

## Success Criteria Checklist

- [ ] **SC-001**: Content explains ROS 2's middleware role
- [ ] **SC-002**: Nodes, Topics, Services explained with examples
- [ ] **SC-003**: rclpy integration concepts covered
- [ ] **SC-004**: URDF components explained
- [ ] **SC-005**: Content renders in Docusaurus without errors
- [ ] **SC-006**: Reading time ~15-25 minutes
- [ ] **SC-007**: Code snippets syntax-highlighted

## Common Issues

| Issue | Solution |
|-------|----------|
| Chapter not in sidebar | Check `sidebar_position` in frontmatter |
| Build fails | Verify frontmatter YAML syntax |
| Broken links | Use relative paths: `./02-core-primitives.md` |
| Code not highlighted | Ensure language identifier after triple backticks |

## Next Steps

After Module 1 content is complete:

1. Run `/sp.tasks` to generate implementation tasks
2. Run `/sp.implement` to execute content creation
3. Validate against success criteria
4. Prepare for RAG ingestion integration
