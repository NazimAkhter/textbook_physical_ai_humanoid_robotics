# Chapter Contract: Chapter 2 - Voice-to-Action Pipelines

**Chapter ID**: `02-voice-to-action`
**Sidebar Label**: "Voice-to-Action Pipelines"
**Sidebar Position**: 2
**Target Word Count**: 700-900 words
**Priority**: P2

## Learning Objectives

From spec.md User Story 2:
- "Describe the voice-to-action pipeline from speech input to robot execution"
- "Explain the role of speech recognition, intent extraction, and ROS 2 action mapping"
- "Identify challenges in real-time speech processing for robotics contexts"

## Chapter Structure

### 1. Introduction (~150 words)
**Chunk**: `<!-- chunk:introduction -->`

**Content Requirements**:
- Connect to Chapter 1: "Chapter 1 introduced VLA as the cognitive layer. Now we dive into voice-to-action pipelines, the interface enabling natural language robot control."
- Motivate voice control: hands-free operation, accessibility, natural interaction
- Preview pipeline stages: speech recognition → NLU → intent extraction → action mapping

**Integration Points**:
- Reference VLA system architecture from Chapter 1
- Preview ROS 2 action servers (detailed in this chapter)

---

### 2. Speech Recognition for Robotics (~250 words)
**Chunk**: `<!-- chunk:speech-recognition -->`

**Content Requirements**:
- Define speech recognition: audio stream → text transcription (FR-004)
- Service abstraction pattern (architectural decision 2 from plan.md):
  - Technology-agnostic interface (not tied to specific vendor)
  - Examples: OpenAI Whisper, Google Cloud STT, Azure Speech Services
  - Whisper as concrete example (open-source, multilingual)
- Key characteristics for robotics:
  - Latency: <500ms target (from research.md)
  - Robustness: noise handling (factory, home environments)
  - Accuracy: domain-specific vocabulary (robot commands)
- ROS 2 integration: speech recognition as ROS 2 service node

**Data Model Entities**: Speech Recognition Module (entity 3)

**Research References**: OpenAI Whisper (from research.md)

**Diagram**: Speech recognition component box
```
Audio Input (microphone)
    ↓
Speech Recognition Service (<500ms)
    ↓
Text Output ("Go to the kitchen")
```

---

### 3. Natural Language Understanding (~200 words)
**Chunk**: `<!-- chunk:nlu -->`

**Content Requirements**:
- Define NLU: text → structured understanding
- NLU tasks for robotics:
  - Intent classification (navigate, grasp, detect, etc.)
  - Entity recognition (locations, objects, attributes)
  - Parameter extraction (target pose, object properties)
- Example transformation:
  - Input: "Pick up the red cup on the table"
  - Output: `{intent: "grasp", object: "cup", color: "red", location: "table"}`
- Lightweight NLU models for low latency (<100ms from research.md)
- Distinction from LLM planning (covered in Chapter 3): NLU extracts intent, LLM decomposes tasks

**Data Model Entities**: Intent Extractor (entity 4)

---

### 4. Intent to Action Mapping (~250 words)
**Chunk**: `<!-- chunk:action-mapping -->`

**Content Requirements**:
- Define Action Mapper: intent → ROS 2 action goals (FR-005)
- ROS 2 action server recap (from Module 1):
  - Action interface: Goal, Feedback, Result
  - Examples: NavigateToPose (Nav2), Grasp, DetectObjects (Isaac ROS)
- Mapping examples:
  - `navigate` intent → `NavigateToPose` action (Nav2 from Module 3)
  - `grasp` intent → `Grasp` action (manipulation controller)
  - `detect` intent → `DetectObjects` action (Isaac ROS from Module 3)
- Validation: check action feasibility, parameter constraints
- Execution: send goals to ROS 2 action servers

**Data Model Entities**: Action Mapper, ROS 2 Action Servers (entities 5, 8)

**Integration Points**:
- ROS 2 actions (Module 1)
- Nav2 NavigateToPose (Module 3)
- Isaac ROS perception actions (Module 3)

**Code Snippet Location**: ROS 2 action client for voice-controlled navigation (detailed below)

---

### 5. Real-Time Constraints (~250 words)
**Chunk**: `<!-- chunk:real-time-constraints -->`

**Content Requirements**:
- Latency budget breakdown (FR-006, from research.md):
  - Speech recognition: <500ms
  - NLU/Intent extraction: <100ms
  - LLM planning: 1-3s (acceptable for high-level commands)
  - **Total command-to-action**: <4s acceptable
- Robustness challenges:
  - Background noise (industrial environments, household sounds)
  - Accent and dialect variations
  - Ambiguous commands (require clarification)
  - Continuous listening vs. wake-word activation
- Trade-offs:
  - Accuracy vs. latency (cloud STT vs. on-device models)
  - Flexibility vs. predictability (scripted commands vs. LLM planning)

**Table**: Latency budget comparison
| Component | Target Latency | Purpose |
|-----------|----------------|---------|
| Speech Recognition | <500ms | Audio → text |
| NLU/Intent | <100ms | Text → structured intent |
| LLM Planning | 1-3s | Task decomposition |
| Total | <4s | Command → action start |

---

### 6. Voice-to-Action Pipeline Architecture (~200 words)
**Chunk**: `<!-- chunk:pipeline-architecture -->`

**Content Requirements**:
- Complete pipeline diagram showing all stages (FR-004, FR-005)
- Data transformations at each stage
- Integration with ROS 2 ecosystem
- Feedback loop for error handling

**Diagram Required**: Complete voice-to-action pipeline (ASCII)
```
User Voice Command
    ↓
┌─────────────────────────────────┐
│ Speech Recognition (<500ms)     │ → Audio stream → Text
└─────────────┬───────────────────┘
              ↓ "Go to kitchen"
┌─────────────────────────────────┐
│ NLU / Intent Extraction (<100ms)│ → Text → Structured intent
└─────────────┬───────────────────┘
              ↓ {command: "navigate", target: "kitchen"}
┌─────────────────────────────────┐
│ Action Mapper                   │ → Intent → ROS 2 action goal
└─────────────┬───────────────────┘
              ↓ NavigateToPose(kitchen_pose)
┌─────────────────────────────────┐
│ ROS 2 Action Server (Nav2)      │ → Action goal → Physical execution
└─────────────┬───────────────────┘
              ↓
         Robot Motion
              ↓
       Execution Result
         (Success/Failure)
```

**Data Model Entities**: Voice-to-Action Pipeline (entity 2, complete)

---

### 7. Key Takeaways (~50 words)
**Chunk**: `<!-- chunk:key-takeaways -->`

**Content Requirements** (3-5 bullet points):
- Voice-to-action pipeline transforms speech to robot execution through 4 stages
- Speech recognition and NLU must meet strict latency constraints (<600ms combined)
- Action Mapper translates intents to ROS 2 action servers from Module 1 and Module 3
- Real-time robotics requires trade-offs between accuracy, latency, and robustness
- Service abstraction pattern enables technology flexibility

---

### 8. Next Steps (~50 words)

**Content Requirements**:
- Transition to Chapter 3: "Voice-to-action pipelines handle simple commands well. Chapter 3 explores LLM-based cognitive planning for complex multi-step tasks requiring reasoning and adaptation."

---

## Code Snippet Requirement

**Type**: ROS 2 action client for voice-controlled navigation

**Disclaimer** (REQUIRED):
```python
# Conceptual ROS 2 Action Client for Voice-Controlled Navigation
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
```

**Content** (15-30 lines):
- ROS 2 action client sending NavigateToPose goal
- Voice command → intent → ROS 2 action goal flow
- Feedback and result handling
- Syntactically valid Python (rclpy)

**Example**:
```python
# Conceptual ROS 2 Action Client for Voice-Controlled Navigation
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://docs.ros.org/en/humble/Tutorials/

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class VoiceControlledNavigator:
    def __init__(self, node):
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def execute_voice_command(self, intent):
        # Intent from voice pipeline: {command: "navigate", target: "kitchen"}
        if intent['command'] == 'navigate':
            target_location = intent['target']

            # Create ROS 2 action goal
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position = self.get_location_pose(target_location)

            # Send goal to Nav2 action server
            self.nav_client.wait_for_server()
            future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
            return future

    def feedback_cb(self, feedback_msg):
        # Monitor navigation progress
        distance_remaining = feedback_msg.feedback.distance_remaining
        self.node.get_logger().info(f'Distance remaining: {distance_remaining:.2f}m')
```

---

## Diagram Requirements

**Diagram 1: Speech Recognition Component** (ASCII, simple)
- Input: Audio stream
- Processing: Speech recognition (<500ms)
- Output: Text transcription
- Show latency constraint annotation

**Diagram 2: Complete Voice-to-Action Pipeline** (ASCII, detailed)
- All 4 stages: Speech Recognition → NLU → Action Mapper → ROS 2 Actions
- Data transformations at each stage
- Latency annotations
- Feedback loop from execution results

---

## Module Cross-References

**Required References**:
- **Module 1 (ROS 2)**: ROS 2 action interface (Goal, Feedback, Result), action clients/servers
- **Module 3 (Nav2)**: NavigateToPose action for navigation
- **Module 3 (Isaac ROS)**: DetectObjects action for perception
- **Chapter 1**: VLA system architecture, cognitive layer positioning

**Reference Pattern**:
```markdown
The Action Mapper translates intents to ROS 2 action server calls (Module 1)
using navigation capabilities from Nav2 (Module 3) and perception from Isaac ROS (Module 3).
```

---

## RAG Chunk Map

All major sections MUST be wrapped in chunk comments:
- `<!-- chunk:introduction -->`
- `<!-- chunk:speech-recognition -->`
- `<!-- chunk:nlu -->`
- `<!-- chunk:action-mapping -->`
- `<!-- chunk:real-time-constraints -->`
- `<!-- chunk:pipeline-architecture -->`
- `<!-- chunk:key-takeaways -->`

---

## Quality Checklist (Chapter 2 Specific)

Before marking chapter complete, verify:
- [ ] **Word count**: 700-900 words (excluding frontmatter, code, diagrams, tables)
- [ ] **Frontmatter**: Complete with id, title, sidebar_label, sidebar_position, description, keywords, learning_objectives
- [ ] **Chunk comments**: All 7 major sections wrapped in `<!-- chunk:name -->` tags
- [ ] **Code snippet**: ROS 2 action client with disclaimer, syntactically valid Python (rclpy)
- [ ] **Diagrams**: 2 ASCII diagrams (speech recognition, complete pipeline) with latency annotations
- [ ] **Module cross-references**: References to Modules 1 and 3 (at least 3 references)
- [ ] **Key Takeaways**: 3-5 bullet points summarizing chapter
- [ ] **Next Steps**: Smooth transition to Chapter 3
- [ ] **Tone**: Systems-level, explanatory, non-marketing
- [ ] **Learning objectives**: All 3 objectives achievable from chapter content
- [ ] **Build test**: `npm run build` passes in frontend/
- [ ] **Speech recognition pipeline**: Clear explanation of audio → text transformation (FR-004)
- [ ] **Voice-to-action translation**: Complete pipeline from speech to execution (FR-005)
- [ ] **Real-time constraints**: Latency budget and robustness challenges documented (FR-006)
- [ ] **Latency table**: Present with all 4 pipeline stages

---

## Acceptance Criteria (from spec.md)

**User Story 2 Success**:
- ✅ Reader can identify pipeline stages: speech recognition, NLU, intent extraction, action execution
- ✅ Reader can explain how speech commands map to ROS 2 action servers
- ✅ Reader can describe how real-time requirements affect voice interface design

**Independent Test**: After reading Chapter 2, reader should be able to:
1. Describe each stage of the voice-to-action pipeline
2. Explain latency budget allocation (<4s total)
3. Map example voice command to ROS 2 action goal
4. Identify robustness challenges for robotics speech interfaces

---

## Implementation Notes

**Priority**: P2 - Second chapter to implement

**Dependencies**: Chapter 1 (VLA foundations), quickstart.md, data-model.md

**Estimated Complexity**: Medium - pipeline stages, latency analysis, ROS 2 integration code

**Integration Risk**: Low - builds on established ROS 2 action pattern from Module 1

**Follow-up Tasks** (after Chapter 2 complete):
- Validate against quality checklist
- Build and preview with Docusaurus
- Proceed to Chapter 3 (Cognitive Planning)
