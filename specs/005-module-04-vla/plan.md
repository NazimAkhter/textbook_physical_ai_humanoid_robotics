# Implementation Plan: Module 4 - Vision-Language-Action (VLA)

**Branch**: `005-module-04-vla` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-module-04-vla/spec.md`

## Summary

Create Module 4 educational content covering Vision-Language-Action (VLA) systems as the cognitive layer enabling humanoid robots to understand natural language commands and execute complex behaviors. Four chapters explain VLA foundations, voice-to-action pipelines, LLM-based cognitive planning, and end-to-end autonomous integration—building on ROS 2, Digital Twins, and Isaac from Modules 1-3. Content is systems-focused and conceptual (600-1000 words per chapter, 2500-4000 total), with illustrative code snippets rather than implementation tutorials.

## Technical Context

**Language/Version**: Markdown for Docusaurus 3.x
**Primary Dependencies**: Docusaurus, React (for any interactive components)
**Storage**: N/A (static documentation)
**Testing**: Manual editorial review, markdown linting, link validation, comprehension exercises
**Target Platform**: Web (Docusaurus on Vercel)
**Project Type**: Documentation/Educational content
**Performance Goals**: All pages load <3s, mobile-responsive
**Constraints**: 2500-4000 words total (600-1000 per chapter), systems-level tone, conceptual only
**Scale/Scope**: 4 chapters, references to VLA research (RT-1, PaLM-E, SayCan)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-Driven Development | PASS | Following `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` sequence |
| II. Educational Efficacy | PASS | Content connects VLA concepts to ROS 2/Isaac workflows; bridges LLM theory to robot action |
| III. Documentation-Agent Integration | PASS | Book content serves as single source of truth for RAG chatbot; VLA chapter will be indexed |
| IV. Reproducibility by Default | PASS | Conceptual ROS 2 action interface and LLM prompt snippets will be syntactically valid; no full implementations per FR-017 |
| V. Technical Standards | PASS | Docusaurus framework, mobile-responsive, Vercel deployment |
| VI. Constraints & Compliance | PASS | Using Claude Code + Spec-Kit Plus; strict scope boundaries (no speech-to-text guides, no prompt engineering, no safety discussions per FR-017) |

**Content Structure Alignment**: Module 4 is "Vision-Language-Action (VLA) Systems" in constitution—title matches exactly.

## Project Structure

### Documentation (this feature)

```text
specs/005-module-04-vla/
├── plan.md              # This file
├── research.md          # Phase 0: VLA architecture, LLM integration patterns research
├── data-model.md        # Phase 1: Content entities (VLA components, pipelines)
├── quickstart.md        # Phase 1: Writing guidelines for VLA chapters
├── contracts/           # Phase 1: Chapter outline contracts
│   ├── ch01-vla-foundations.md
│   ├── ch02-voice-to-action.md
│   ├── ch03-cognitive-planning.md
│   └── ch04-end-to-end-integration.md
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (Docusaurus content)

```text
frontend/docs/
├── intro.md
├── module-01-ros2-nervous-system/
├── module-02-digital-twin/
├── module-03-nvidia-isaac/
└── module-04-vla/                      # NEW - This feature
    ├── _category_.json                 # Module metadata
    ├── 01-vla-foundations.md           # Chapter 1: VLA Systems Overview
    ├── 02-voice-to-action.md           # Chapter 2: Voice-to-Action Pipelines
    ├── 03-cognitive-planning.md        # Chapter 3: LLM-Based Planning
    └── 04-end-to-end-integration.md    # Chapter 4: Complete Autonomous Pipeline
```

**Structure Decision**: Following established Docusaurus pattern from Modules 1-3. Each module is a directory with `_category_.json` for sidebar configuration and numbered markdown files for chapters. Module 4 completes the Physical AI educational progression.

## Architecture Overview

### VLA Pipeline Architecture (Chapter 1 & 4 Core Concept)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                Vision-Language-Action (VLA) Pipeline                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Human Input Layer                                                       │
│  ┌──────────────┐                                                        │
│  │ Voice/Text   │                                                        │
│  │   Input      │                                                        │
│  └──────┬───────┘                                                        │
│         │                                                                │
│         ▼                                                                │
│  ┌──────────────────────────────────────────────────────┐               │
│  │         Speech Recognition (Chapter 2)                │               │
│  │  • Audio → Text transcription                         │               │
│  │  • Latency: <500ms target for real-time interaction  │               │
│  │  • Robustness: noise handling, accents               │               │
│  └──────────────────┬───────────────────────────────────┘               │
│                     │                                                    │
│                     ▼                                                    │
│  ┌──────────────────────────────────────────────────────┐               │
│  │    Natural Language Understanding (NLU)               │               │
│  │  • Intent extraction from transcribed text            │               │
│  │  • Entity recognition (objects, locations, actions)   │               │
│  │  • Command classification                             │               │
│  └──────────────────┬───────────────────────────────────┘               │
│                     │                                                    │
│                     ▼                                                    │
│  ┌──────────────────────────────────────────────────────┐               │
│  │      LLM-Based Cognitive Planner (Chapter 3)          │               │
│  │                                                        │               │
│  │  Input Context:                                        │               │
│  │  • Natural language goal                               │               │
│  │  • Environmental state (from Isaac ROS perception)     │               │
│  │  • Robot capabilities (ROS 2 action servers available) │               │
│  │  • Task history                                        │               │
│  │                                                        │               │
│  │  LLM Processing:                                       │               │
│  │  • Decompose high-level goal into sub-tasks           │               │
│  │  • Sequence planning with dependencies                │               │
│  │  • Constraint satisfaction (safety, feasibility)      │               │
│  │                                                        │               │
│  │  Output:                                               │               │
│  │  • Ordered sequence of ROS 2 action calls              │               │
│  │  • Parameters for each action                          │               │
│  └──────────────────┬───────────────────────────────────┘               │
│                     │                                                    │
│                     ▼                                                    │
│  ┌──────────────────────────────────────────────────────┐               │
│  │        Action Mapper (Chapter 2)                      │               │
│  │  • Translate planned actions to ROS 2 action goals    │               │
│  │  • Validate action feasibility                        │               │
│  │  • Execute action sequences                            │               │
│  └──────────────────┬───────────────────────────────────┘               │
│                     │                                                    │
│                     ▼                                                    │
│  ┌──────────────────────────────────────────────────────┐               │
│  │      ROS 2 Action Servers (Module 1)                  │               │
│  │  • Navigation actions (Nav2 from Module 3)            │               │
│  │  • Manipulation actions (grasp, place, etc.)          │               │
│  │  • Perception queries (Isaac ROS from Module 3)       │               │
│  └──────────────────┬───────────────────────────────────┘               │
│                     │                                                    │
│                     ▼                                                    │
│  ┌──────────────────────────────────────────────────────┐               │
│  │         Physical Execution Layer                      │               │
│  │  • ROS 2 Control (Module 1)                           │               │
│  │  • Isaac perception feedback loop (Module 3)          │               │
│  │  • Digital twin validation (Module 2)                 │               │
│  └──────────────────────────────────────────────────────┘               │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### System Integration Points (Chapter 4)

```
┌─────────────────────────────────────────────────────────────────────┐
│                   Complete Autonomous Humanoid Stack                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Cognitive Layer (Module 4 - NEW)                                   │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │ VLA System: Voice → LLM Planning → Action Commands      │        │
│  └────────────────────┬────────────────────────────────────┘        │
│                       │                                              │
│                       ▼                                              │
│  ═══════════════════════════════════════════════════════════        │
│                       │                                              │
│                       ▼                                              │
│  Perception & Navigation Layer (Module 3)                            │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │ Isaac ROS (GPU perception) + Nav2 (path planning)       │        │
│  └────────────────────┬────────────────────────────────────┘        │
│                       │                                              │
│                       ▼                                              │
│  ═══════════════════════════════════════════════════════════        │
│                       │                                              │
│                       ▼                                              │
│  Simulation & Training Layer (Module 2 + Module 3)                   │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │ Digital Twins (Gazebo/Unity) + Isaac Sim (Synthetic Data)│       │
│  └────────────────────┬────────────────────────────────────┘        │
│                       │                                              │
│                       ▼                                              │
│  ═══════════════════════════════════════════════════════════        │
│                       │                                              │
│                       ▼                                              │
│  Control & Communication Layer (Module 1)                            │
│  ┌─────────────────────────────────────────────────────────┐        │
│  │ ROS 2 (nodes, topics, actions, services) + Control      │        │
│  └────────────────────┬────────────────────────────────────┘        │
│                       │                                              │
│                       ▼                                              │
│                  Actuators                                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Architectural Decisions

### Decision 1: VLA System Decomposition

**Context**: VLA systems can be architected as monolithic agents or modular pipelines.

**Options Considered**:
1. **Monolithic agent**: Single LLM handles speech recognition, planning, and action mapping
2. **Modular pipeline**: Separate components for speech, NLU, planning, action mapping
3. **Hybrid**: Specialized modules with LLM orchestration

**Decision**: **Modular pipeline** architecture (Option 2)

**Rationale**:
- **Educational Clarity**: Modular design makes each component's role explicit, improving learning outcomes
- **Systems-Level Understanding**: Aligns with course goal of teaching integration patterns, not just LLM usage
- **Real-World Alignment**: Production robotics systems use modular architectures for reliability and debugging
- **Testability**: Each module can be validated independently (matches user story structure)
- **Scope Management**: Separates concerns aligning with spec user stories (US2: voice-to-action, US3: cognitive planning)

**Trade-offs**:
- More integration complexity vs. simpler monolithic approach
- Requires explaining inter-component communication (addressed in Chapter 4)
- Benefits outweigh costs for educational objectives

**Implementation Impact**:
- Chapter 2 focuses on voice-to-action pipeline stages
- Chapter 3 focuses on LLM cognitive planning in isolation
- Chapter 4 shows how all components integrate

### Decision 2: Speech Interface Abstraction Level

**Context**: Speech recognition can be treated as external service (Whisper API) or embedded component.

**Options Considered**:
1. **Service abstraction**: Generic speech-to-text interface, implementation-agnostic
2. **Whisper-specific**: Reference OpenAI Whisper explicitly
3. **ROS 2 native**: Focus on audio_common ROS 2 packages

**Decision**: **Service abstraction** (Option 1) with Whisper as example reference

**Rationale**:
- **Technology Agnostic**: Aligns with spec requirement (FR-017: no speech-to-text implementation guides)
- **Longevity**: Content remains valid as speech recognition technology evolves
- **Scope Appropriate**: Systems-level understanding more valuable than API-specific details
- **Educational Pattern**: Matches Module 3's approach (Isaac ROS concepts, not CUDA programming)
- **Practical**: Readers can apply concepts to any speech recognition system (Whisper, Google STT, Azure)

**Trade-offs**:
- Less concrete than implementation-specific examples
- Requires readers to apply concepts to chosen technology
- Mitigated by: conceptual code showing interface contracts, references to Whisper/alternatives in Further Reading

**Implementation Impact**:
- Chapter 2 describes speech recognition pipeline abstractly
- Code snippets show ROS 2 topic/service interfaces, not Whisper API calls
- Quickstart.md includes guideline: "Reference technologies as examples, not prescriptive solutions"

### Decision 3: LLM Role in Capstone (Chapter 4)

**Context**: Chapter 4 demonstrates end-to-end autonomous behavior. LLM's role can vary in scope.

**Options Considered**:
1. **Task decomposition only**: LLM breaks goals into sub-tasks, execution is scripted
2. **Full autonomy**: LLM handles planning, execution monitoring, replanning on failure
3. **Cognitive planning with feedback**: LLM plans, perceives results, adapts (middle ground)

**Decision**: **Cognitive planning with feedback** (Option 3)

**Rationale**:
- **Realistic Scope**: Full autonomy (Option 2) requires extensive error handling beyond Module 4 scope
- **Demonstrates Integration**: Shows how LLM uses Isaac perception (Module 3) and Nav2 (Module 3) feedback
- **Educational Value**: Illustrates perception-cognition-action loop without overcomplicating
- **Spec Alignment**: Matches FR-010 (end-to-end walkthrough) and FR-011 (Isaac/Nav2 integration)
- **Success Criteria**: Enables SC-004 (trace natural language command through full system)

**Trade-offs**:
- More complex than pure decomposition (Option 1)
- Less comprehensive than full autonomy (Option 2)
- Balanced for educational objectives

**Implementation Impact**:
- Chapter 4 example: "Go to the kitchen and bring me a cup"
  1. LLM plans: Navigate to kitchen → Detect cup → Grasp cup → Navigate to user → Hand over
  2. Execution uses Nav2 (Module 3) for navigation, Isaac ROS (Module 3) for detection
  3. LLM receives feedback after each step (success/failure)
  4. Demonstrates replanning if cup not found (adaptive behavior)

### Decision 4: Chapter Structure Alignment

**Context**: Spec allows "3-4 chapters" but user stories suggest 4 distinct learning objectives.

**Options Considered**:
1. **3 chapters**: Combine voice-to-action and cognitive planning
2. **4 chapters**: One chapter per user story (P1-P4)
3. **5 chapters**: Split end-to-end into integration + capstone

**Decision**: **4 chapters** - one per user story (Option 2)

**Rationale**:
- **1:1 Mapping**: Each user story (P1-P4) becomes independently testable chapter
- **Word Count**: 600-1000 words per chapter × 4 = 2400-4000 (within 2500-4000 spec requirement)
- **Consistency**: Matches Module 3's 4-chapter structure (established pattern)
- **Learning Progression**: Clear path from foundations → voice → planning → integration
- **Testability**: Each chapter's learning objectives map directly to acceptance scenarios

**Trade-offs**:
- Shorter chapters (600-1000 words) vs. longer combined chapters
- More granular structure vs. fewer, denser chapters
- Benefits: clearer learning objectives, easier comprehension checks

**Implementation Impact**:
- Chapter 1 (600-800 words): VLA foundations, architecture overview
- Chapter 2 (700-900 words): Voice-to-action pipeline, speech recognition concepts
- Chapter 3 (700-900 words): LLM cognitive planning, prompting patterns
- Chapter 4 (800-1000 words): End-to-end integration, capstone walkthrough

## Content Design Patterns

### Code Snippet Strategy

Following Module 3's approach, all code snippets MUST be:

1. **Conceptual, not executable tutorials** (per FR-015, FR-017)
2. **Syntactically valid** (per Constitution IV)
3. **Commented with disclaimers**:
   ```python
   # Conceptual ROS 2 action client for voice-controlled navigation
   # Shows integration pattern - NOT executable without full system setup
   # For implementation details, see: [ROS 2 Actions Documentation]

   from rclpy.action import ActionClient
   from nav2_msgs.action import NavigateToPose

   class VoiceNavigationClient:
       def __init__(self, node):
           self._action_client = ActionClient(
               node,
               NavigateToPose,
               'navigate_to_pose'
           )

       def send_goal_from_speech(self, location_name):
           """
           Translate speech command to navigation goal.
           Example: 'Go to the kitchen' → NavigateToPose goal
           """
           # Intent extraction would happen in NLU layer
           # This shows the action mapping stage
           goal_pose = self.location_db.lookup(location_name)

           goal_msg = NavigateToPose.Goal()
           goal_msg.pose = goal_pose

           return self._action_client.send_goal_async(goal_msg)
   ```

4. **Referenced to external resources** for implementation:
   - OpenAI Whisper documentation
   - ROS 2 action server tutorials
   - RT-1, PaLM-E, SayCan research papers

### Diagram Style

Following Module 3's ASCII/text-based approach for consistency:
- VLA pipeline diagrams (boxes and arrows)
- System integration diagrams (layered architecture)
- Data flow diagrams (speech → intent → action → execution)

### Learning Objectives Format

Each chapter frontmatter includes:
```yaml
---
id: vla-foundations
title: "Vision-Language-Action Systems"
sidebar_label: "VLA Foundations"
sidebar_position: 1
description: "Understand how vision, language, and action converge to enable natural language robot control."
keywords: ["vla", "vision-language-action", "llm", "robotics", "cognitive planning"]
learning_objectives:
  - "Explain VLA as the convergence of computer vision, NLP, and robot control"
  - "Describe VLA's position in the humanoid robotics stack"
  - "Identify the three core components (vision, language, action) and their interactions"
---
```

## Complexity Tracking

> No Constitution violations. All complexity justified within educational content scope.

| Principle | Compliance | Notes |
|-----------|------------|-------|
| Spec-Driven Development | ✅ | Following full SDD workflow |
| Educational Efficacy | ✅ | Connects LLM concepts to ROS 2 actions |
| Documentation-Agent Integration | ✅ | Content will be RAG-indexed |
| Reproducibility | ✅ | Conceptual code syntactically valid |
| Technical Standards | ✅ | Docusaurus, mobile-responsive |
| Constraints & Compliance | ✅ | No scope creep, no implementation guides |

## Phase 0: Research (Next Step)

Research areas to be documented in `research.md`:

1. **VLA System Architectures**
   - Review RT-1 (Robotics Transformer), PaLM-E, SayCan papers
   - Identify common architectural patterns
   - Document modular vs. monolithic trade-offs

2. **Speech-to-Action Pipeline Patterns**
   - Survey speech recognition interfaces (Whisper, Google STT, Azure)
   - Identify abstraction patterns for ROS 2 integration
   - Document latency and robustness challenges

3. **LLM Cognitive Planning Approaches**
   - Review LLM prompting strategies for task decomposition
   - Identify context requirements (environmental state, robot capabilities)
   - Document differences between scripted and adaptive planning

4. **End-to-End Integration Patterns**
   - Survey systems combining perception (Isaac ROS), navigation (Nav2), and LLM planning
   - Identify feedback loop architectures
   - Document capstone example structure (realistic but achievable in 800-1000 words)

5. **Educational Content Best Practices**
   - Review Module 3 writing patterns for consistency
   - Identify diagram styles that worked well
   - Document code snippet disclaimer patterns

**Research Completion Criteria**:
- All architectural decisions validated with research citations
- Chapter structures validated against learning objectives
- Code snippet patterns identified
- Diagram styles selected
- Reference material curated (research papers, documentation links)

## Phase 1: Design & Contracts (After Research)

Artifacts to generate:

1. **data-model.md**: VLA system components and relationships
   - Vision-Language-Action System entity
   - Voice-to-Action Pipeline stages
   - Cognitive Planner context and outputs
   - Action Mapper interfaces

2. **contracts/ch01-vla-foundations.md**: Chapter 1 outline
   - Introduction connecting to Modules 1-3
   - VLA definition and motivation
   - System architecture overview
   - Component breakdown (vision, language, action)
   - Position in humanoid robotics stack
   - Key takeaways and next steps

3. **contracts/ch02-voice-to-action.md**: Chapter 2 outline
   - Introduction connecting to Chapter 1
   - Speech recognition pipeline
   - Natural language understanding
   - Intent extraction patterns
   - ROS 2 action mapping
   - Latency and robustness challenges
   - Code snippet: ROS 2 action client interface
   - Key takeaways and next steps

4. **contracts/ch03-cognitive-planning.md**: Chapter 3 outline
   - Introduction connecting to Chapter 2
   - LLM-based task decomposition
   - Context requirements (state, capabilities, history)
   - Prompting patterns for robot planning
   - Scripted vs. adaptive planning comparison
   - Code snippet: LLM prompt structure
   - Key takeaways and next steps

5. **contracts/ch04-end-to-end-integration.md**: Chapter 4 outline
   - Introduction: capstone integration
   - Complete pipeline walkthrough (speech → execution)
   - Integration with Isaac ROS perception
   - Integration with Nav2 navigation
   - Feedback loops and replanning
   - Full system architecture diagram
   - Example: "Go to kitchen, bring cup" scenario
   - Key takeaways and course conclusion

6. **quickstart.md**: Writing guidelines
   - Word count targets (600-1000 per chapter)
   - Tone guidelines (systems-level, explanatory, non-marketing)
   - Code snippet disclaimer patterns
   - Diagram style (ASCII/text-based)
   - Reference citation format
   - RAG chunk comment patterns
   - Quality checklist

## Success Criteria

Module 4 plan is complete and ready for `/sp.tasks` when:

- ✅ All architectural decisions documented with rationale
- ✅ VLA pipeline architecture defined with component boundaries
- ✅ System integration points with Modules 1-3 explicitly mapped
- ✅ Chapter structure (4 chapters, one per user story) validated
- ✅ Code snippet strategy aligned with FR-015, FR-017
- ✅ Constitution compliance verified (no violations)
- ✅ Research areas identified for Phase 0
- ✅ Phase 1 artifacts list complete
- ✅ Capstone narrative structure defined (Chapter 4)
- ✅ Success criteria measurable and testable

**Next Command**: Create `research.md` with VLA architecture findings, then proceed to Phase 1 design artifacts before running `/sp.tasks`.
