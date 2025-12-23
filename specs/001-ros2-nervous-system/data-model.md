# Data Model: Module 1 – The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-nervous-system`
**Date**: 2025-12-14
**Status**: Complete

## Overview

This document defines the content entities, their attributes, and relationships for Module 1. Since this is educational content (not a software application), the data model describes content structure rather than database schemas.

## Content Entities

### 1. Module

Represents a top-level educational unit within the book.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `id` | string | Unique identifier | `module-01-ros2-nervous-system` |
| `title` | string | Display title | "ROS 2 as the Robotic Nervous System" |
| `order` | integer | Navigation position | 1 (fixed per constitution) |
| `description` | string | Brief summary for SEO/RAG | Max 160 characters |
| `chapters` | Chapter[] | Ordered list of chapters | Exactly 4 chapters |
| `word_count_target` | range | Target total word count | 2,500–4,000 |

### 2. Chapter

Represents a section within a module covering a specific topic.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `id` | string | Unique identifier | `01-ros2-overview`, `02-core-primitives`, etc. |
| `title` | string | Display title | From spec content structure |
| `order` | integer | Position within module | 1-4 (fixed) |
| `file_path` | string | Relative file location | `docs/module-01-ros2-nervous-system/{id}.md` |
| `learning_objectives` | string[] | What reader will learn | 2-4 objectives per chapter |
| `word_count_target` | integer | Target word count | ~625-1000 words each |
| `concepts` | Concept[] | Key concepts introduced | At least 2 per chapter |
| `code_snippets` | CodeSnippet[] | Illustrative examples | At least 1 per chapter |
| `diagram_descriptions` | DiagramDescription[] | Visual explanations | At least 1 per chapter |
| `summary` | string | Key takeaways section | Required per FR-009 |
| `prerequisites` | string | Prior chapters needed | Optional, shown in callout |

### 3. Concept

Represents a theoretical idea explained in the content.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `id` | string | Unique identifier | Slugified name (e.g., `dds-middleware`) |
| `name` | string | Display name | Human-readable (e.g., "DDS Middleware") |
| `definition` | string | Explanation text | 1-3 sentences |
| `first_defined_in` | Chapter | Where concept is introduced | Chapter reference |
| `related_concepts` | Concept[] | Linked concepts | 0-5 related concepts |
| `analogy` | string | Biological/real-world comparison | Optional, for nervous system theme |

### 4. CodeSnippet

Represents an illustrative code example.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `id` | string | Unique identifier | Auto-generated from title |
| `title` | string | Descriptive title | Shown above code block |
| `language` | enum | Programming language | `python`, `xml`, `bash` |
| `purpose` | string | What the snippet illustrates | 1 sentence |
| `code` | string | Actual code content | Syntactically valid |
| `is_executable` | boolean | Can be run as-is | `false` for all Module 1 snippets |
| `disclaimer` | string | Non-executable notice | Required per FR-005 |

### 5. DiagramDescription

Represents a textual description of a visual concept.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `id` | string | Unique identifier | Slugified caption |
| `type` | enum | Diagram category | `architecture`, `data-flow`, `comparison`, `hierarchy` |
| `caption` | string | Figure title | Descriptive, 5-10 words |
| `description` | string | Textual explanation | What the diagram shows |
| `elements` | string[] | Key visual elements | List of components |
| `chapter` | Chapter | Where diagram appears | Chapter reference |

### 6. Frontmatter

Represents metadata at the top of each Markdown file.

| Attribute | Type | Description | Constraints |
|-----------|------|-------------|-------------|
| `id` | string | Document identifier | Matches filename slug |
| `title` | string | Page title | Used in browser tab |
| `sidebar_label` | string | Short navigation label | Max 25 characters |
| `sidebar_position` | integer | Navigation order | Matches chapter order |
| `description` | string | SEO/RAG description | Max 160 characters |
| `keywords` | string[] | Discovery tags | 3-7 keywords |
| `learning_objectives` | string[] | Chapter objectives | 2-4 items |

## Entity Relationships

```
Module 1:N Chapter
Chapter 1:N Concept
Chapter 1:N CodeSnippet
Chapter 1:N DiagramDescription
Concept N:N Concept (related_concepts)
Chapter N:1 Frontmatter
```

## Content Inventory: Module 1

### Chapter 1: ROS 2 as the Robotic Nervous System

| Entity | Count | Items |
|--------|-------|-------|
| Concepts | 4 | ROS 2 Middleware, Message Passing, DDS, Real-Time Communication |
| Code Snippets | 0 | N/A (conceptual chapter) |
| Diagram Descriptions | 2 | Robot architecture overview, Nervous system analogy |

### Chapter 2: ROS 2 Core Primitives

| Entity | Count | Items |
|--------|-------|-------|
| Concepts | 5 | Node, Topic, Service, Action, Executor |
| Code Snippets | 2 | Node lifecycle example, Pub/sub pattern |
| Diagram Descriptions | 2 | Communication patterns comparison, Humanoid data flow |

### Chapter 3: Bridging Python AI Agents to ROS 2

| Entity | Count | Items |
|--------|-------|-------|
| Concepts | 4 | rclpy, Publisher, Subscriber, Callback |
| Code Snippets | 3 | Simple node, Publisher setup, Subscriber setup |
| Diagram Descriptions | 1 | AI agent to ROS 2 integration |

### Chapter 4: Understanding URDF for Humanoid Robots

| Entity | Count | Items |
|--------|-------|-------|
| Concepts | 4 | URDF, Link, Joint, Transmission |
| Code Snippets | 2 | Link definition, Joint definition |
| Diagram Descriptions | 2 | Humanoid joint hierarchy, Link-joint relationship |

## Validation Rules

1. **Module completeness**: All 4 chapters must exist with required attributes
2. **Word count**: Total content 2,500–4,000 words; no chapter <500 words
3. **Concept coverage**: Each concept defined exactly once, on first use
4. **Code quality**: All snippets syntactically valid (linted)
5. **Frontmatter presence**: Every Markdown file has complete frontmatter
6. **Cross-references**: Related concepts must reference existing concepts
7. **Order consistency**: `sidebar_position` matches spec-defined chapter order
