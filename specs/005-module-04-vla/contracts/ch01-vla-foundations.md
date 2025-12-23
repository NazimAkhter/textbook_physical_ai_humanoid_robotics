# Chapter Contract: Chapter 1 - VLA Foundations

**Chapter ID**: `01-vla-foundations`
**Sidebar Label**: "VLA Foundations"
**Sidebar Position**: 1
**Target Word Count**: 600-800 words
**Priority**: P1 (MVP)

## Learning Objectives

From spec.md User Story 1:
- "Explain VLA as the convergence of computer vision, NLP, and robot control"
- "Describe VLA's position in the humanoid robotics stack"
- "Identify the three core components (vision, language, action) and their interactions"

## Chapter Structure

### 1. Introduction (~150 words)
**Chunk**: `<!-- chunk:introduction -->`

**Content Requirements**:
- Connect to Modules 1-3: "Module 1 covered ROS 2, Module 2 covered Digital Twins, Module 3 covered Isaac ROS perception and Nav2 navigation"
- Introduce cognitive layer gap: robots can see and move, but need reasoning/language understanding
- Preview VLA as the missing cognitive layer

**Integration Points**:
- Reference ROS 2 (Module 1)
- Reference Isaac ROS perception (Module 3)
- Reference Nav2 navigation (Module 3)

---

### 2. What is Vision-Language-Action? (~200 words)
**Chunk**: `<!-- chunk:vla-definition -->`

**Content Requirements**:
- Define VLA as convergence of three disciplines (FR-001):
  - Computer vision: understanding environment
  - Natural language processing: understanding human intent
  - Robot control: executing physical actions
- Explain WHY convergence is necessary (not just WHAT it is)
- Provide concrete example: "Go to kitchen and bring me a cup"
  - Vision: detect cup, navigate obstacles
  - Language: understand "kitchen", "bring", "cup"
  - Action: navigate, grasp, return

**Data Model Entities**: VLA System (entity 1)

**Research References**: RT-1, PaLM-E, SayCan (from research.md)

---

### 3. VLA in the Humanoid Robotics Stack (~200 words)
**Chunk**: `<!-- chunk:vla-stack-position -->`

**Content Requirements**:
- Position VLA as cognitive layer sitting above perception/navigation (FR-002)
- Show vertical stack:
  - Layer 4 (top): VLA Cognitive Planning
  - Layer 3: Isaac ROS Perception, Nav2 Navigation
  - Layer 2: ROS 2 Communication (topics, actions, services)
  - Layer 1: Robot Hardware (sensors, actuators)
- Explain data flow upward (sensor data → perception → planning) and downward (plans → actions → control)

**Diagram Required**: ASCII vertical stack with 4 layers

**Integration Points**:
- Isaac ROS (Module 3) provides perception inputs
- Nav2 (Module 3) receives navigation commands
- ROS 2 actions (Module 1) provide communication

---

### 4. VLA System Architecture (~250 words)
**Chunk**: `<!-- chunk:vla-architecture -->`

**Content Requirements**:
- Present modular VLA pipeline (architectural decision 1 from plan.md)
- Component breakdown (FR-003):
  - Speech Recognition: audio → text
  - Natural Language Understanding: text → intent
  - Cognitive Planner: intent → action sequence
  - Action Mapper: action sequence → ROS 2 actions
  - Execution & Feedback: ROS 2 actions → physical execution → perception feedback
- Emphasize modularity (NOT monolithic end-to-end learning)

**Diagram Required**: Box-and-arrow VLA pipeline (ASCII)
```
Voice Input → Speech Recognition → NLU → Cognitive Planner
                                              ↓
                                         Action Mapper
                                              ↓
                                      ROS 2 Action Servers
                                              ↓
                                        Execution
                                              ↓
                                      Perception Feedback → (loops back to Planner)
```

**Data Model Entities**: VLA System, Voice-to-Action Pipeline (entities 1-2)

---

### 5. Three Core Components (~300 words)
**Chunk**: `<!-- chunk:three-components -->`

**Content Requirements**:
- **Vision Component**:
  - Object detection, localization, scene understanding
  - Provided by Isaac ROS (Module 3)
  - Example: detecting cup, identifying obstacles

- **Language Component**:
  - Speech recognition, intent extraction, task decomposition
  - LLM-based cognitive planning
  - Example: "bring me a cup" → navigate(kitchen), detect(cup), grasp(cup), navigate(user)

- **Action Component**:
  - ROS 2 action servers (Nav2, manipulation)
  - Physical execution through ROS 2 Control (Module 1)
  - Example: NavigateToPose, Grasp, Place actions

**Interactions**:
- Vision informs Language: perception results provide context for planning
- Language directs Action: LLM plans specify which actions to execute
- Action feeds Vision: execution results trigger perception updates

**Data Model Entities**: Speech Recognition Module, Cognitive Planner, ROS 2 Action Servers (entities 3, 6, 8)

---

### 6. Data Flow in VLA Systems (~200 words)
**Chunk**: `<!-- chunk:data-flow -->`

**Content Requirements**:
- Trace complete loop for "Go to kitchen":
  1. Speech input captured
  2. Speech recognition → "Go to kitchen" (text)
  3. Intent extraction → {command: "navigate", target: "kitchen"}
  4. LLM planning → action sequence
  5. Action mapping → NavigateToPose(kitchen_pose)
  6. ROS 2 execution → robot moves
  7. Perception feedback → confirm arrival or trigger replanning
- Emphasize feedback loop (not open-loop execution)

**Data Model Entities**: Complete pipeline (all 9 entities referenced)

---

### 7. Key Takeaways (~50 words)
**Chunk**: `<!-- chunk:key-takeaways -->`

**Content Requirements** (3-5 bullet points):
- VLA converges vision, language, and action for cognitive robotics
- VLA sits as cognitive layer above perception/navigation in humanoid stack
- Modular pipeline architecture enables composability and debugging
- Feedback loop between perception and planning enables adaptive behavior
- Builds on ROS 2 (Module 1), Digital Twins (Module 2), Isaac/Nav2 (Module 3)

---

### 8. Next Steps (~50 words)

**Content Requirements**:
- Transition to Chapter 2: "Now that we understand VLA foundations, Chapter 2 explores voice-to-action pipelines, showing how speech commands translate to robot execution with real-time constraints."

---

## Code Snippet Requirement

**Type**: Conceptual VLA system abstract interface

**Disclaimer** (REQUIRED):
```python
# Conceptual VLA System Interface for Educational Purposes
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: [Reference to ROS 2 Actions documentation]
```

**Content** (10-30 lines):
- Abstract base class or interface showing VLA system structure
- Methods: process_speech_input(), plan_task(), execute_actions(), handle_feedback()
- NOT a full implementation (conceptual only)
- Syntactically valid Python

**Example**:
```python
# Conceptual VLA System Interface for Educational Purposes
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://docs.ros.org/en/humble/Tutorials/

class VLASystem:
    def __init__(self, speech_recognizer, cognitive_planner, action_mapper):
        self.speech = speech_recognizer
        self.planner = cognitive_planner
        self.mapper = action_mapper

    def process_command(self, audio_input):
        # Speech recognition: audio → text
        text = self.speech.recognize(audio_input)

        # Cognitive planning: text → action sequence
        action_sequence = self.planner.decompose_task(text)

        # Action mapping: action sequence → ROS 2 actions
        for action in action_sequence:
            ros_action_goal = self.mapper.to_ros_action(action)
            result = self.execute_action(ros_action_goal)

            if not result.success:
                # Trigger replanning on failure
                action_sequence = self.planner.replan(result.feedback)

        return result
```

---

## Diagram Requirements

**Diagram 1: VLA Stack Position** (ASCII, vertical)
- 4 layers: VLA, Perception/Navigation, ROS 2, Hardware
- Show data flow arrows
- Label each layer with representative components

**Diagram 2: VLA Pipeline Architecture** (ASCII, horizontal)
- Box-and-arrow flow: Speech → NLU → Planner → Mapper → Execution
- Show feedback loop from Execution back to Planner
- Annotate arrows with data types (e.g., "text", "intent", "action goal")

---

## Module Cross-References

**Required References**:
- **Module 1 (ROS 2)**: ROS 2 actions, topics, services, ROS 2 Control
- **Module 2 (Digital Twins)**: Simulation environments for VLA testing
- **Module 3 (Isaac ROS)**: Perception inputs (object detection, localization)
- **Module 3 (Nav2)**: Navigation capabilities (NavigateToPose action)

**Reference Pattern** (from quickstart.md):
```markdown
The VLA Cognitive Planner receives perception inputs from Isaac ROS (Module 3)
and coordinates navigation actions through Nav2 (Module 3) using ROS 2 action
servers (Module 1).
```

---

## RAG Chunk Map

All major sections MUST be wrapped in chunk comments:
- `<!-- chunk:introduction -->`
- `<!-- chunk:vla-definition -->`
- `<!-- chunk:vla-stack-position -->`
- `<!-- chunk:vla-architecture -->`
- `<!-- chunk:three-components -->`
- `<!-- chunk:data-flow -->`
- `<!-- chunk:key-takeaways -->`

---

## Quality Checklist (Chapter 1 Specific)

Before marking chapter complete, verify:
- [ ] **Word count**: 600-800 words (excluding frontmatter, code, diagrams)
- [ ] **Frontmatter**: Complete with id, title, sidebar_label, sidebar_position, description, keywords, learning_objectives
- [ ] **Chunk comments**: All 7 major sections wrapped in `<!-- chunk:name -->` tags
- [ ] **Code snippet**: Includes disclaimer, syntactically valid Python, shows VLA system interface pattern
- [ ] **Diagrams**: 2 ASCII diagrams (VLA stack, VLA pipeline) with clear labels
- [ ] **Module cross-references**: References to Modules 1-3 where relevant (at least 4 references)
- [ ] **Key Takeaways**: 3-5 bullet points summarizing chapter
- [ ] **Next Steps**: Smooth transition to Chapter 2
- [ ] **Tone**: Systems-level, explanatory, non-marketing (no "revolutionary", "cutting-edge")
- [ ] **Learning objectives**: All 3 objectives achievable from chapter content
- [ ] **Build test**: `npm run build` passes in frontend/
- [ ] **VLA definition**: Clear explanation of vision-language-action convergence (FR-001)
- [ ] **Stack position**: VLA positioned as cognitive layer above perception/navigation (FR-002)
- [ ] **Systems architecture**: Modular pipeline diagram showing components and data flow (FR-003)

---

## Acceptance Criteria (from spec.md)

**User Story 1 Success**:
- ✅ Reader can explain why combining vision, language, and action is necessary
- ✅ Reader can describe how VLA builds upon navigation and perception systems
- ✅ Reader can trace how visual observations and language commands translate into robot actions

**Independent Test**: After reading Chapter 1, reader should be able to:
1. Define VLA as convergence of three disciplines
2. Position VLA in the humanoid robotics stack (Layer 4)
3. Identify the three core components and their interactions
4. Trace data flow from speech input to physical execution

---

## Implementation Notes

**Priority**: P1 (MVP) - This is the first deliverable chapter

**Dependencies**: Requires quickstart.md and data-model.md (already created)

**Estimated Complexity**: Medium - foundational concepts, multiple diagrams required

**Integration Risk**: Low - no external dependencies beyond Modules 1-3 references

**Follow-up Tasks** (after Chapter 1 complete):
- Validate against quality checklist
- Build and preview with Docusaurus
- Proceed to Chapter 2 (Voice-to-Action Pipelines)
