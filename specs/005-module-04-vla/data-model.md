# Data Model: Module 4 - Vision-Language-Action (VLA)

**Purpose**: Define VLA system components, their relationships, and data flow for educational content

**Note**: This is a conceptual model for educational content, not a database schema or implementation spec.

## Core Entities

### 1. Vision-Language-Action (VLA) System

**Description**: The complete integration of visual perception, natural language understanding, and robot action execution

**Key Attributes**:
- System architecture (modular pipeline)
- Component interfaces (perception, language, action)
- Integration points with ROS 2, Isaac, Nav2

**Relationships**:
- Contains: Voice-to-Action Pipeline, Cognitive Planner, Action Mapper
- Integrates with: Isaac ROS (Module 3), Nav2 (Module 3), ROS 2 Control (Module 1)

**Content Coverage**: Chapter 1 (VLA Foundations)

---

### 2. Voice-to-Action Pipeline

**Description**: Processing chain converting speech input to robot action execution

**Key Attributes**:
- Pipeline stages (speech recognition, NLU, intent extraction, action mapping)
- Real-time latency constraints (<4s total)
- Robustness requirements (noise handling, accents)

**Sub-Components**:
- **Speech Recognition Module**: Audio → text transcription
- **Intent Extractor**: Text → actionable commands
- **Action Mapper**: Commands → ROS 2 action goals

**Relationships**:
- Feeds into: Cognitive Planner
- Outputs to: ROS 2 Action Servers

**Content Coverage**: Chapter 2 (Voice-to-Action Pipelines)

---

### 3. Speech Recognition Module

**Description**: Component converting voice input to text for natural language understanding

**Key Attributes**:
- Input: Audio stream (microphone)
- Output: Transcribed text
- Latency: <500ms target
- Implementation examples: Whisper, Google STT, Azure Speech

**Abstraction Level**: Service interface (technology-agnostic)

**Relationships**:
- Part of: Voice-to-Action Pipeline
- Feeds: Intent Extractor

**Content Coverage**: Chapter 2 (Speech Recognition section)

---

### 4. Intent Extractor

**Description**: System component identifying actionable robot commands from natural language input

**Key Attributes**:
- Input: Transcribed text
- Output: Structured intent (command type, entities, parameters)
- Processing: NLU models, entity recognition
- Latency: <100ms target

**Example Transformations**:
- "Go to the kitchen" → `{command: "navigate", target: "kitchen"}`
- "Pick up the red cup" → `{command: "grasp", object: "cup", attribute: "red"}`

**Relationships**:
- Part of: Voice-to-Action Pipeline
- Receives from: Speech Recognition Module
- Feeds: Action Mapper

**Content Coverage**: Chapter 2 (NLU and Intent Extraction section)

---

### 5. Action Mapper

**Description**: Interface translating extracted intents to ROS 2 action server calls

**Key Attributes**:
- Input: Structured intent
- Output: ROS 2 action goals (NavigateToPose, Grasp, etc.)
- Validation: Checks action feasibility, parameters
- Execution: Sends goals to ROS 2 action servers

**Example Mappings**:
- `navigate` intent → `NavigateToPose` action (Nav2)
- `grasp` intent → `Grasp` action (manipulation controller)
- `detect` intent → `DetectObjects` action (Isaac ROS)

**Relationships**:
- Part of: Voice-to-Action Pipeline
- Receives from: Intent Extractor (simple commands) OR Cognitive Planner (complex tasks)
- Sends to: ROS 2 Action Servers

**Content Coverage**: Chapter 2 (Intent to Action Mapping section)

---

### 6. Cognitive Planner (LLM-Based)

**Description**: LLM-based system that decomposes high-level natural language goals into executable ROS 2 action sequences

**Key Attributes**:
- Input: Natural language goal + context
- Processing: LLM inference (CoT prompting, few-shot learning)
- Output: Ordered sequence of ROS 2 action calls with parameters
- Latency: 1-3s acceptable for high-level planning

**Context Requirements**:
- **Environmental State**: Robot location, visible objects, obstacles
- **Robot Capabilities**: Available action servers, preconditions
- **Task History**: Previous actions, outcomes, user feedback

**Planning Approaches**:
- Chain-of-Thought (CoT): Explicit reasoning traces
- Few-Shot Learning: Example command→action mappings
- ReAct: Reasoning + Acting with feedback

**Relationships**:
- Part of: VLA System
- Receives from: Intent Extractor (high-level goals), Perception (Isaac ROS)
- Sends to: Action Mapper
- Feedback from: Action Execution Results

**Content Coverage**: Chapter 3 (LLM-Based Cognitive Planning)

---

### 7. LLM Context

**Description**: Environmental state, robot capabilities, and task history provided to language models for planning

**Key Attributes**:
- **Environmental State**:
  - Current robot pose (from localization)
  - Detected objects and positions (from Isaac ROS)
  - Obstacle map (from Nav2 costmap)
  - Scene semantics (from segmentation)

- **Robot Capabilities**:
  - Available ROS 2 action servers list
  - Action preconditions (e.g., "grasp requires object detection")
  - Physical constraints (reach radius, payload limits)

- **Task History**:
  - Executed actions log
  - Success/failure outcomes
  - User corrections and feedback

**Format**: Structured text or embeddings for LLM consumption

**Relationships**:
- Provided to: Cognitive Planner
- Updated by: Perception (Isaac ROS), Action Execution feedback

**Content Coverage**: Chapter 3 (Context Requirements section)

---

### 8. ROS 2 Action Servers

**Description**: Standard ROS 2 action interfaces providing robot capabilities (navigation, manipulation, perception)

**Key Action Types**:
- **Navigation**: `NavigateToPose` (Nav2 from Module 3)
- **Manipulation**: `Grasp`, `Place`, `MoveToJointPosition`
- **Perception**: `DetectObjects`, `TrackObject` (Isaac ROS from Module 3)

**Attributes**:
- Goal: Action-specific parameters (e.g., target pose for navigation)
- Feedback: Progress updates during execution
- Result: Success/failure status, outcome data

**Relationships**:
- Receives goals from: Action Mapper
- Provides feedback to: Cognitive Planner (for replanning)
- Executes via: ROS 2 Control (Module 1)

**Content Coverage**: Chapter 2 (ROS 2 Action Mapping), Chapter 4 (Integration)

---

### 9. Perception Feedback Loop

**Description**: Continuous integration of Isaac ROS perception outputs with VLA planning and execution

**Key Data Flows**:
- **Localization** (Visual SLAM) → LLM Context (robot pose)
- **Object Detection** → LLM Context (visible objects)
- **Costmap Updates** → Navigation feasibility checks
- **Execution Results** → LLM replanning triggers

**Feedback Types**:
- **Success**: Action completed, continue to next step
- **Partial Success**: Action completed with deviations, replan
- **Failure**: Action blocked/failed, invoke recovery or replan

**Relationships**:
- Connects: Isaac ROS (Module 3), Cognitive Planner, Action Execution
- Enables: Adaptive planning, error recovery

**Content Coverage**: Chapter 4 (Integration sections, Capstone example)

---

## System Integration Diagram

```
┌─────────────────────────────────────────────────────────┐
│              VLA System (Module 4)                      │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Voice/Text Input                                       │
│       ↓                                                 │
│  ┌──────────────────────────────────┐                  │
│  │  Speech Recognition Module       │                  │
│  └──────────┬───────────────────────┘                  │
│             ↓                                           │
│  ┌──────────────────────────────────┐                  │
│  │  Intent Extractor (NLU)          │                  │
│  └──────────┬───────────────────────┘                  │
│             ↓                                           │
│  ┌──────────────────────────────────────────────┐      │
│  │  Cognitive Planner (LLM)                      │      │
│  │  • Receives: Intent + LLM Context             │      │
│  │  • Processes: CoT, few-shot, ReAct            │      │
│  │  • Outputs: Action sequence                   │      │
│  └──────────┬────────────────────────────────────┘      │
│             ↓                                           │
│  ┌──────────────────────────────────┐                  │
│  │  Action Mapper                   │                  │
│  │  • Intent → ROS 2 action goals   │                  │
│  └──────────┬───────────────────────┘                  │
│             ↓                                           │
└─────────────┼───────────────────────────────────────────┘
              ↓
    ┌─────────────────────┐
    │ ROS 2 Action Servers│ (Module 1 + Module 3)
    │ • Nav2 (navigation) │
    │ • Isaac ROS (detect)│
    │ • Control (grasp)   │
    └─────────┬───────────┘
              ↓
         Execution
              ↓
         ┌────────┐
         │Feedback│ ← Isaac ROS Perception (Module 3)
         └────┬───┘
              ↓
         LLM Context (updates for replanning)
```

## Data Flow Example: "Go to kitchen and bring me a cup"

1. **Speech Input**: User speaks command
2. **Speech Recognition**: "Go to kitchen and bring me a cup" (text)
3. **Intent Extraction**: High-level goal identified
4. **Cognitive Planning** (LLM decomposes):
   - Sub-task 1: Navigate to kitchen
   - Sub-task 2: Detect cup
   - Sub-task 3: Grasp cup
   - Sub-task 4: Navigate back to user
   - Sub-task 5: Hand over cup
5. **Action Mapping & Execution**:
   - Step 1: `NavigateToPose(kitchen)` → Nav2 → Success
   - Step 2: `DetectObjects(cup)` → Isaac ROS → Success (cup found)
   - Step 3: `Grasp(cup_id)` → Manipulation → Success
   - Step 4: `NavigateToPose(user_location)` → Nav2 → Success
   - Step 5: `HandOver()` → Manipulation → Success
6. **Feedback Loop**: Each step result updates LLM Context, triggers replanning if failure

## Entity Relationships Summary

| Entity | Provides To | Receives From |
|--------|-------------|---------------|
| Speech Recognition | Intent Extractor | User audio input |
| Intent Extractor | Cognitive Planner, Action Mapper | Speech Recognition |
| Cognitive Planner | Action Mapper | Intent Extractor, LLM Context |
| Action Mapper | ROS 2 Action Servers | Cognitive Planner, Intent Extractor |
| ROS 2 Action Servers | Execution results, Feedback | Action Mapper |
| LLM Context | Cognitive Planner | Isaac ROS, Execution feedback |
| Perception Feedback | LLM Context, Cognitive Planner | Isaac ROS, Action results |

## Content Mapping

| Chapter | Primary Entities Covered |
|---------|-------------------------|
| Chapter 1 | VLA System, System Integration |
| Chapter 2 | Voice-to-Action Pipeline, Speech Recognition, Intent Extractor, Action Mapper, ROS 2 Action Servers |
| Chapter 3 | Cognitive Planner, LLM Context, Planning Approaches |
| Chapter 4 | Perception Feedback Loop, Complete Integration, Capstone Example |

**Next Steps**: Use these entities to structure chapter contracts and quickstart writing guidelines
