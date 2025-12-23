# Chapter Contract: Chapter 3 - LLM-Based Cognitive Planning

**Chapter ID**: `03-cognitive-planning`
**Sidebar Label**: "Cognitive Planning"
**Sidebar Position**: 3
**Target Word Count**: 700-900 words
**Priority**: P3

## Learning Objectives

From spec.md User Story 3:
- "Explain how LLMs translate abstract goals into concrete action sequences"
- "Describe the role of prompting and context in robot planning"
- "Identify differences between scripted command execution and LLM-based planning"

## Chapter Structure

### 1. Introduction (~150 words)
**Chunk**: `<!-- chunk:introduction -->`

**Content Requirements**:
- Connect to Chapter 2: "Chapter 2 covered voice-to-action pipelines for simple commands. Complex multi-step tasks require cognitive planning—breaking abstract goals into executable action sequences."
- Motivate LLM planning: flexibility, adaptation, reasoning over environmental context
- Preview chapter focus: LLMs for task decomposition, context requirements, prompting patterns, scripted vs adaptive comparison

**Integration Points**:
- Reference voice-to-action pipeline from Chapter 2
- Preview Isaac ROS perception inputs (context for planning)

---

### 2. LLMs for Robot Task Planning (~250 words)
**Chunk**: `<!-- chunk:llm-planning -->`

**Content Requirements**:
- Define cognitive planning: high-level goal → ordered sequence of robot actions (FR-007)
- Why LLMs for planning:
  - Natural language understanding (user intent)
  - Commonsense reasoning (task ordering, preconditions)
  - Generalization (handle novel phrasings)
- Example decomposition:
  - Goal: "Go to kitchen and bring me a cup"
  - LLM output:
    1. navigate(kitchen)
    2. detect_object(cup)
    3. grasp(cup)
    4. navigate(user_location)
    5. hand_over(cup)
- Contrast with scripted approaches (table in section 4)

**Data Model Entities**: Cognitive Planner (entity 6)

**Research References**: SayCan (grounding language in affordances), CoT prompting

**Architectural Decision**: Cognitive planning with feedback (decision 3 from plan.md)

---

### 3. Context Requirements (~250 words)
**Chunk**: `<!-- chunk:context-requirements -->`

**Content Requirements**:
- Define LLM context: environmental state + robot capabilities + task history (FR-008)

**Environmental State**:
- Robot location (from localization - Module 3)
- Detected objects and positions (from Isaac ROS - Module 3)
- Obstacle map (from Nav2 costmap - Module 3)
- Scene semantics (from segmentation)

**Robot Capabilities**:
- Available ROS 2 action servers list
- Action preconditions (e.g., "grasp requires object detection first")
- Physical constraints (reach radius, payload limits)

**Task History**:
- Previously executed actions
- Success/failure outcomes
- User corrections and feedback

**Context Format**:
- Structured text for LLM consumption
- Example context snippet:
```
Current Location: living_room
Visible Objects: [cup (x=2.3, y=1.1), table (x=2.0, y=1.0)]
Available Actions: navigate, detect_object, grasp, place
Previous Actions: navigate(kitchen) - SUCCESS
```

**Data Model Entities**: LLM Context (entity 7)

**Integration Points**:
- Isaac ROS perception (Module 3) provides environmental state
- Nav2 costmap (Module 3) provides obstacle information
- ROS 2 action servers (Module 1) define available capabilities

---

### 4. Prompting Patterns for Robotics (~250 words)
**Chunk**: `<!-- chunk:prompting-patterns -->`

**Content Requirements**:
- Three key prompting strategies (from research.md):

**Chain-of-Thought (CoT)**:
- Pattern: "Let's break this down step-by-step..."
- Benefit: Explicit reasoning traces improve task decomposition
- Example prompt structure

**Few-Shot In-Context Learning**:
- Pattern: Provide 2-3 examples of command → action mappings
- Benefit: Grounds LLM in robot-specific action vocabulary
- Example:
```
Command: "Pick up the red block"
Actions: navigate_to_object(red_block), grasp(red_block)

Command: "Go to the kitchen"
Actions: navigate_to_location(kitchen)

Command: [user's command]
Actions:
```

**ReAct (Reasoning + Acting)**:
- Pattern: Interleave reasoning, action execution, observation
- Benefit: Enables replanning based on execution feedback
- Flow: Plan → Execute → Observe → Replan if needed

**Code Snippet Location**: LLM prompt structure example (detailed below)

**Research References**: CoT prompting papers, ReAct framework

---

### 5. Scripted vs Adaptive Planning (~250 words)
**Chunk**: `<!-- chunk:scripted-vs-adaptive -->`

**Content Requirements**:
- Comparison table contrasting scripted and LLM-based approaches (FR-009)

**Table Required**:
| Aspect | Scripted Command Execution | LLM-Based Adaptive Planning |
|--------|----------------------------|------------------------------|
| **Flexibility** | Fixed command mappings | Handles novel phrasing |
| **Robustness** | Fails on unseen commands | Generalizes from examples |
| **Transparency** | Explicit action sequences | Reasoning traces optional |
| **Failure Handling** | Predefined error recovery | Can replan based on feedback |
| **Computational Cost** | Minimal (lookup table) | Moderate (LLM inference 1-3s) |
| **Suitable For** | Well-defined command set | Open-ended natural language |

**When to use each**:
- Scripted: safety-critical tasks, deterministic behavior required
- LLM-based: exploratory tasks, unstructured environments, novel scenarios

**Hybrid approach**: Use scripted for low-level primitives, LLM for high-level coordination

---

### 6. Coordinating Robot Primitives (~200 words)
**Chunk**: `<!-- chunk:coordinating-primitives -->`

**Content Requirements**:
- How LLMs coordinate navigation, perception, and manipulation primitives
- Example coordination:
  - Navigation: navigate_to_location() calls Nav2 (Module 3)
  - Perception: detect_objects() calls Isaac ROS (Module 3)
  - Manipulation: grasp() and place() call manipulation controller (Module 1)
- Sequencing constraints:
  - Preconditions: grasp requires object detection first
  - Postconditions: navigate after grasp to avoid dropping object
- Feedback integration:
  - Success: continue to next step
  - Partial success: adapt parameters and retry
  - Failure: trigger replanning with updated context

**Data Model Entities**: Cognitive Planner, ROS 2 Action Servers, Perception Feedback Loop (entities 6, 8, 9)

**Integration Points**:
- Nav2 (Module 3) for navigation primitives
- Isaac ROS (Module 3) for perception primitives
- ROS 2 Control (Module 1) for manipulation primitives

---

### 7. Key Takeaways (~50 words)
**Chunk**: `<!-- chunk:key-takeaways -->`

**Content Requirements** (3-5 bullet points):
- LLMs decompose high-level goals into executable ROS 2 action sequences
- Effective planning requires environmental state, robot capabilities, and task history
- CoT prompting, few-shot learning, and ReAct enable robust task planning
- Adaptive planning handles novel commands but adds computational cost vs. scripted
- LLMs coordinate navigation (Nav2), perception (Isaac ROS), and manipulation primitives

---

### 8. Next Steps (~50 words)

**Content Requirements**:
- Transition to Chapter 4: "We've explored VLA foundations, voice-to-action pipelines, and cognitive planning. Chapter 4 integrates all components into a complete autonomous humanoid system, demonstrating end-to-end execution with a capstone example."

---

## Code Snippet Requirement

**Type**: LLM prompt structure for robot task planning

**Disclaimer** (REQUIRED):
```python
# Conceptual LLM Prompt Structure for Robot Task Planning
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: LangChain, OpenAI API documentation
```

**Content** (20-30 lines):
- Complete prompt structure including system message, context, few-shot examples, user command
- Show how environmental state and capabilities are formatted
- Syntactically valid Python (string formatting)

**Example**:
```python
# Conceptual LLM Prompt Structure for Robot Task Planning
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://platform.openai.com/docs/

def create_robot_planning_prompt(user_command, environmental_context, robot_capabilities):
    system_prompt = """You are a robot task planner. Given a natural language command,
    decompose it into a sequence of executable robot actions.

    Available actions: navigate(location), detect_object(object_name), grasp(object_id), place(location)
    """

    # Environmental context from Isaac ROS perception
    context_text = f"""
    Current Robot State:
    - Location: {environmental_context['location']}
    - Visible Objects: {environmental_context['objects']}
    - Available Actions: {robot_capabilities['actions']}
    """

    # Few-shot examples (in-context learning)
    examples = """
    Example 1:
    Command: "Pick up the red block"
    Plan: 1. detect_object(red_block)
          2. grasp(red_block)

    Example 2:
    Command: "Go to the kitchen"
    Plan: 1. navigate(kitchen)
    """

    # Complete prompt
    full_prompt = f"{system_prompt}\n\n{context_text}\n\n{examples}\n\nCommand: {user_command}\nPlan:"

    return full_prompt

# Usage with LLM API
prompt = create_robot_planning_prompt(
    user_command="Go to kitchen and bring me a cup",
    environmental_context={'location': 'living_room', 'objects': ['table', 'chair']},
    robot_capabilities={'actions': ['navigate', 'detect_object', 'grasp', 'place']}
)
```

---

## Diagram Requirements

**Diagram 1: LLM Cognitive Planning Flow** (ASCII)
```
User Command + Environmental Context
    ↓
┌─────────────────────────────────────────┐
│ LLM Cognitive Planner                   │
│ • Chain-of-Thought reasoning            │
│ • Few-shot in-context learning          │
│ • Task decomposition                    │
└─────────────┬───────────────────────────┘
              ↓
    Ordered Action Sequence
    [navigate(kitchen), detect(cup), grasp(cup), navigate(user)]
              ↓
┌─────────────────────────────────────────┐
│ Action Mapper (Chapter 2)               │
└─────────────┬───────────────────────────┘
              ↓
    ROS 2 Action Goals (Nav2, Isaac, Manipulation)
              ↓
         Execution
              ↓
    ┌─────────────────┐
    │ Feedback Loop   │
    │ Success/Failure │
    └────────┬────────┘
             ↓
      Replanning if needed (ReAct pattern)
```

**Diagram 2: Context Inputs to LLM** (ASCII box diagram)
```
┌─────────────────────────────────────────────┐
│         LLM Context Inputs                  │
├─────────────────────────────────────────────┤
│                                             │
│  Environmental State (from Isaac ROS):      │
│  • Robot location (localization)            │
│  • Detected objects (object detection)      │
│  • Obstacle map (costmap)                   │
│                                             │
│  Robot Capabilities (from ROS 2):           │
│  • Available action servers                 │
│  • Action preconditions/effects             │
│  • Physical constraints                     │
│                                             │
│  Task History:                              │
│  • Previous actions executed                │
│  • Success/failure outcomes                 │
│  • User feedback                            │
│                                             │
└─────────────────────────────────────────────┘
            ↓
    LLM Cognitive Planner
```

---

## Module Cross-References

**Required References**:
- **Module 1 (ROS 2)**: ROS 2 action servers, action primitives
- **Module 3 (Isaac ROS)**: Perception inputs (object detection, localization)
- **Module 3 (Nav2)**: Navigation primitives, costmap
- **Chapter 2**: Voice-to-action pipeline, Action Mapper

**Reference Pattern**:
```markdown
The LLM Cognitive Planner receives environmental context from Isaac ROS (Module 3),
coordinates navigation through Nav2 (Module 3), and executes actions via ROS 2
action servers (Module 1).
```

---

## RAG Chunk Map

All major sections MUST be wrapped in chunk comments:
- `<!-- chunk:introduction -->`
- `<!-- chunk:llm-planning -->`
- `<!-- chunk:context-requirements -->`
- `<!-- chunk:prompting-patterns -->`
- `<!-- chunk:scripted-vs-adaptive -->`
- `<!-- chunk:coordinating-primitives -->`
- `<!-- chunk:key-takeaways -->`

---

## Quality Checklist (Chapter 3 Specific)

Before marking chapter complete, verify:
- [ ] **Word count**: 700-900 words (excluding frontmatter, code, diagrams, table)
- [ ] **Frontmatter**: Complete with id, title, sidebar_label, sidebar_position, description, keywords, learning_objectives
- [ ] **Chunk comments**: All 7 major sections wrapped in `<!-- chunk:name -->` tags
- [ ] **Code snippet**: LLM prompt structure with disclaimer, syntactically valid Python
- [ ] **Diagrams**: 2 ASCII diagrams (LLM planning flow, context inputs) with clear labels
- [ ] **Comparison table**: Scripted vs. adaptive planning with 6 aspects
- [ ] **Module cross-references**: References to Modules 1 and 3 (at least 4 references)
- [ ] **Key Takeaways**: 3-5 bullet points summarizing chapter
- [ ] **Next Steps**: Smooth transition to Chapter 4
- [ ] **Tone**: Systems-level, explanatory, non-marketing
- [ ] **Learning objectives**: All 3 objectives achievable from chapter content
- [ ] **Build test**: `npm run build` passes in frontend/
- [ ] **LLM task decomposition**: Clear explanation of goal → action sequence translation (FR-007)
- [ ] **Context requirements**: Environmental state, capabilities, history documented (FR-008)
- [ ] **Scripted vs adaptive**: Table comparing both approaches (FR-009)
- [ ] **Research references**: SayCan, CoT prompting, ReAct mentioned

---

## Acceptance Criteria (from spec.md)

**User Story 3 Success**:
- ✅ Reader can describe how LLMs decompose tasks into executable ROS 2 actions
- ✅ Reader can explain how LLMs coordinate navigation, perception, and manipulation primitives
- ✅ Reader can describe what environmental and robot state information LLMs need

**Independent Test**: After reading Chapter 3, reader should be able to:
1. Explain how LLMs translate abstract goals into action sequences
2. List three types of context required for effective planning
3. Compare scripted vs. LLM-based planning trade-offs
4. Describe CoT, few-shot, and ReAct prompting patterns

---

## Implementation Notes

**Priority**: P3 - Third chapter to implement

**Dependencies**: Chapters 1-2, quickstart.md, data-model.md, research.md

**Estimated Complexity**: Medium-High - LLM integration patterns, prompting strategies, coordination logic

**Integration Risk**: Low - conceptual focus, no executable LLM implementation required

**Follow-up Tasks** (after Chapter 3 complete):
- Validate against quality checklist
- Build and preview with Docusaurus
- Proceed to Chapter 4 (End-to-End Integration)
