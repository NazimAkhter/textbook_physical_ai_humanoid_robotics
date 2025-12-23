---
id: cognitive-planning
title: "LLM-Based Cognitive Planning"
sidebar_label: "Cognitive Planning"
sidebar_position: 3
description: "Explore how large language models enable cognitive planning for robots, translating abstract natural language goals into executable action sequences with context-aware reasoning and adaptive replanning."
keywords: ["LLM planning", "cognitive robotics", "task decomposition", "CoT prompting", "few-shot learning", "ReAct", "adaptive planning", "robot reasoning"]
learning_objectives:
  - "Explain how LLMs translate abstract goals into concrete action sequences"
  - "Describe the role of prompting and context in robot planning"
  - "Identify differences between scripted command execution and LLM-based planning"
---

# LLM-Based Cognitive Planning

<!-- chunk:introduction -->

## Introduction

Chapter 2 covered voice-to-action pipelines for simple, direct commands like "go to kitchen" or "stop." However, real-world robotics demands handling **complex multi-step tasks** specified in abstract natural language: "Go to the kitchen and bring me a cup." Such commands require **cognitive planning**—breaking high-level goals into ordered sequences of executable robot actions.

**Large Language Models (LLMs)** enable this capability through:
- **Commonsense reasoning**: Understanding task ordering and preconditions (e.g., detecting a cup before grasping it)
- **Flexibility**: Handling novel phrasings without rigid command syntax
- **Adaptation**: Replanning when execution fails based on environmental feedback

This chapter explores how LLMs perform task decomposition, the environmental and robot context they require, prompting patterns that improve planning quality (Chain-of-Thought, few-shot learning, ReAct), and trade-offs between scripted and adaptive planning approaches.

<!-- /chunk -->

<!-- chunk:llm-planning -->

## LLMs for Robot Task Planning

**Cognitive planning** transforms a high-level natural language goal into an **ordered sequence of robot actions** thatNav2, Isaac ROS, and manipulation controllers can execute.

### Why LLMs for Planning?

Traditional robotics planning relies on symbolic AI (e.g., PDDL planners) or hand-coded logic. LLMs offer distinct advantages:

1. **Natural Language Understanding**: LLMs parse user intent directly from speech/text without requiring formal command syntax
2. **Commonsense Reasoning**: Pre-training on vast text corpora provides knowledge about task ordering, spatial relationships, and object interactions
3. **Generalization**: LLMs handle novel phrasings ("fetch the mug" vs. "bring me a cup") without explicit programming for each variant

### Example Task Decomposition

**Goal**: "Go to the kitchen and bring me a cup"

**LLM Output** (action sequence):
1. `navigate(kitchen)` — Move to kitchen location
2. `detect_object(cup)` — Use Isaac ROS to locate cup
3. `grasp(cup)` — Execute manipulation primitive to pick up cup
4. `navigate(user_location)` — Return to user's current position
5. `hand_over(cup)` — Release cup to user

Each action maps to a ROS 2 action server (from Modules 1 and 3): `navigate` → `NavigateToPose` (Nav2), `detect_object` → `DetectObjects` (Isaac ROS), `grasp` → `Grasp` (manipulation controller).

### LLM Planning vs. Scripted Approaches

Unlike scripted command execution (lookup tables mapping "go to kitchen" → `NavigateToPose`), LLMs:
- **Decompose** multi-step goals rather than executing single commands
- **Reason** about preconditions (detecting before grasping) and sequencing (navigating after grasping to avoid dropping objects)
- **Adapt** plans based on execution feedback (covered in Section 6)

Research systems like **SayCan** (Google, 2022) demonstrate LLM planning grounded in robot affordances—the LLM proposes actions, and a value function filters infeasible plans. **PaLM-E** (2023) integrates visual perception directly into language model planning, enabling multimodal reasoning.

<!-- /chunk -->

<!-- chunk:context-requirements -->

## Context Requirements

Effective LLM planning requires **three types of context**: environmental state, robot capabilities, and task history.

### Environmental State

The LLM must know the robot's surroundings to plan feasible actions:

- **Robot Location**: Current pose from Visual SLAM (Isaac ROS, Module 3)
  - Example: `Current location: living_room (x=1.2, y=0.8)`
- **Detected Objects**: Visible objects and their positions from Isaac ROS object detection
  - Example: `Visible objects: [cup (x=2.3, y=1.1, z=0.8), table (x=2.0, y=1.0)]`
- **Obstacle Map**: Costmap from Nav2 (Module 3) indicating free space and obstacles
- **Scene Semantics**: Semantic segmentation results (e.g., "kitchen counter," "floor," "wall")

Without environmental state, the LLM cannot determine whether the kitchen is reachable, if a cup is present, or if the path is obstructed.

### Robot Capabilities

The LLM needs to know **what actions the robot can perform**:

- **Available Action Servers**: List of ROS 2 actions (`navigate`, `detect_object`, `grasp`, `place`, `follow`)
- **Action Preconditions**: Constraints on action execution
  - Example: `grasp` requires prior `detect_object` success to know target pose
- **Physical Constraints**: Reach radius (e.g., 0.8m), payload limits (e.g., 2kg max), joint angle limits

This prevents the LLM from planning infeasible actions like "grasp object at (5.0, 5.0)" if the object is beyond reach.

### Task History

The LLM benefits from **memory of past interactions**:

- **Previously Executed Actions**: Log of what the robot has already done
  - Example: `Previous actions: navigate(kitchen) - SUCCESS, detect_object(cup) - FAILURE (not found)`
- **Success/Failure Outcomes**: Execution results inform replanning
- **User Feedback**: Corrections like "No, the other cup" refine understanding

### Context Format Example

LLM context is structured as text:

```
Current Robot State:
- Location: living_room
- Visible Objects: [cup (x=2.3, y=1.1), table (x=2.0, y=1.0)]
- Available Actions: navigate, detect_object, grasp, place

Task History:
- navigate(kitchen) → SUCCESS
- detect_object(cup) → FAILURE (cup not visible in kitchen)

User Command: "Bring me the cup from the living room table"
```

This context enables the LLM to plan: "Since the cup detection failed in the kitchen, and the user mentions the living room table, I should navigate back to living_room and search there."

**Integration Points**: Environmental state comes from Isaac ROS perception (Module 3), obstacle information from Nav2 costmap (Module 3), and available actions from ROS 2 action servers (Module 1).

<!-- /chunk -->

<!-- chunk:prompting-patterns -->

## Prompting Patterns for Robotics

LLM planning quality depends heavily on **prompt engineering**. Three patterns are particularly effective for robotics:

### Chain-of-Thought (CoT) Prompting

**Pattern**: Instruct the LLM to "think step-by-step" before generating actions.

**Benefit**: Explicit reasoning traces improve task decomposition by forcing the model to articulate intermediate steps and preconditions.

**Example Prompt**:
```
You are a robot task planner. Given a natural language command, break it down step-by-step into executable actions.

Command: "Go to the kitchen and bring me a cup"

Think step-by-step:
1. First, I need to navigate to the kitchen
2. Then, I must detect where the cup is located
3. Next, I should grasp the cup
4. After grasping, navigate back to the user
5. Finally, hand over the cup

Actions:
1. navigate(kitchen)
2. detect_object(cup)
3. grasp(cup)
4. navigate(user_location)
5. hand_over(cup)
```

CoT prompting has been shown to improve complex reasoning tasks by 20-30% (Wei et al., 2022).

### Few-Shot In-Context Learning

**Pattern**: Provide 2-3 examples of command → action sequence mappings before the user's actual command.

**Benefit**: Grounds the LLM in **robot-specific action vocabulary** and demonstrates desired output format.

**Example Prompt**:
```
You are a robot task planner. Translate commands into action sequences.

Example 1:
Command: "Pick up the red block"
Actions: navigate_to_object(red_block), grasp(red_block)

Example 2:
Command: "Go to the kitchen"
Actions: navigate(kitchen)

Example 3:
Command: "Find the cup and bring it here"
Actions: detect_object(cup), navigate_to_object(cup), grasp(cup), navigate(user_location), hand_over(cup)

Now plan for:
Command: [user's actual command]
Actions:
```

Few-shot learning leverages the LLM's in-context learning ability without retraining, making it practical for robotics applications with limited task-specific data.

### ReAct (Reasoning + Acting)

**Pattern**: Interleave reasoning, action execution, and observation in a feedback loop.

**Benefit**: Enables **adaptive replanning** based on execution results—the LLM observes outcomes and adjusts the plan accordingly.

**Flow**:
1. **Reason**: LLM plans next action based on current context
2. **Act**: Robot executes action via ROS 2 action server
3. **Observe**: Perception updates environmental state
4. **Replan (if needed)**: If action fails, LLM generates alternative plan with updated context

**Example**:
```
Thought: To bring the cup, I first need to navigate to the kitchen
Action: navigate(kitchen)
Observation: Navigation successful, arrived at kitchen

Thought: Now I need to detect the cup's location
Action: detect_object(cup)
Observation: Cup NOT detected (not visible in current view)

Thought: Cup detection failed. I should search the living room instead
Action: navigate(living_room)
Observation: Navigation successful, arrived at living_room

Action: detect_object(cup)
Observation: Cup detected at (x=2.3, y=1.1, z=0.8)

Thought: Cup found. Now grasp it
Action: grasp(cup)
...
```

ReAct (Yao et al., 2023) demonstrates significant improvements in multi-step reasoning tasks by closing the perception-action loop.

<!-- /chunk -->

<!-- chunk:scripted-vs-adaptive -->

## Scripted vs. Adaptive Planning

Robotics systems often use a **hybrid approach**, combining scripted execution for simple commands with LLM-based adaptive planning for complex tasks.

### Comparison Table

| Aspect | Scripted Command Execution | LLM-Based Adaptive Planning |
|--------|----------------------------|------------------------------|
| **Flexibility** | Fixed command mappings | Handles novel phrasing and multi-step tasks |
| **Robustness** | Fails on unseen commands | Generalizes from examples and context |
| **Transparency** | Explicit, deterministic action sequences | Reasoning traces optional but explainable |
| **Failure Handling** | Predefined error recovery rules | Can replan dynamically based on feedback |
| **Computational Cost** | Minimal (lookup table, &lt;10ms) | Moderate (LLM inference 1-3s per planning cycle) |
| **Suitable For** | Well-defined command set, safety-critical tasks | Open-ended natural language, unstructured environments |

### When to Use Each Approach

**Scripted Execution**:
- **Safety-critical operations**: Emergency stop, collision avoidance (deterministic behavior required)
- **Low-latency tasks**: Simple motions ("move forward 1m," "rotate 90°") where &lt;10ms response matters
- **Well-defined commands**: Fixed vocabulary in industrial settings

**LLM-Based Planning**:
- **Exploratory tasks**: Searching for objects, navigating unfamiliar environments
- **Complex multi-step goals**: Household chores ("clean the table"), fetch-and-deliver tasks
- **Novel scenarios**: User requests not seen during development

### Hybrid Architecture

Most production systems use **LLMs for high-level coordination** and **scripted primitives for low-level execution**:

```
User: "Bring me a cup from the kitchen"
    ↓
LLM Planner (adaptive): Decomposes into [navigate, detect, grasp, return]
    ↓
Action Mapper (scripted): Maps navigate → NavigateToPose (Nav2)
                          Maps grasp → Grasp (scripted manipulation controller)
```

This balances flexibility (LLM handles task decomposition) with reliability (scripted primitives execute proven motion sequences).

<!-- /chunk -->

<!-- chunk:coordinating-primitives -->

## Coordinating Robot Primitives

LLMs coordinate navigation, perception, and manipulation primitives by understanding **preconditions**, **sequencing constraints**, and **feedback integration**.

### Primitive Coordination Example

For "Go to kitchen and bring me a cup," the LLM must coordinate:

1. **Navigation** (`navigate(kitchen)`):
   - Calls Nav2 `NavigateToPose` action (Module 3)
   - Precondition: Valid kitchen location in semantic map
   - Postcondition: Robot arrives at kitchen, enabling object detection

2. **Perception** (`detect_object(cup)`):
   - Calls Isaac ROS `DetectObjects` action (Module 3)
   - Precondition: Robot positioned where cup is visible
   - Postcondition: Cup pose known, enabling grasping

3. **Manipulation** (`grasp(cup)`):
   - Calls manipulation controller `Grasp` action (Module 1)
   - Precondition: Object pose from detection, gripper in open state
   - Postcondition: Cup grasped securely, safe to navigate

4. **Return Navigation** (`navigate(user_location)`):
   - Calls Nav2 `NavigateToPose` again
   - Precondition: Cup grasped (postcondition of step 3)
   - Postcondition: Robot arrives at user, ready for handover

### Sequencing Constraints

LLMs must respect **temporal ordering**:
- **Preconditions**: Cannot grasp before detecting (no target pose)
- **Postconditions**: Must navigate after grasping carefully (avoid dropping object by sudden motions)
- **Safety constraints**: Cannot navigate with gripper open if holding delicate items

The LLM learns these constraints from:
- **Few-shot examples** showing correct orderings
- **Action descriptions** in prompts (e.g., "grasp requires prior object detection")
- **Failure feedback** (if grasp fails because object wasn't detected, LLM learns to insert detection step)

### Feedback Integration

After each action, execution results inform planning:

- **Success**: Continue to next planned action
  - Example: `navigate(kitchen) → SUCCESS` → proceed to `detect_object(cup)`

- **Partial Success**: Adapt parameters and retry
  - Example: `grasp(cup) → PARTIAL (gripper collision)` → retry with adjusted approach angle

- **Failure**: Trigger replanning with updated context
  - Example: `detect_object(cup) → FAILURE (not found)` → LLM replans: "Search living room instead" or "Ask user for cup location"

**ReAct pattern** (Section 4) enables this feedback loop, allowing the LLM to observe outcomes and adjust plans mid-execution rather than executing blindly.

**Integration with ROS 2 Ecosystem**: The LLM coordinates primitives provided by Nav2 (Module 3), Isaac ROS (Module 3), and ROS 2 Control (Module 1), demonstrating the composability of the VLA stack covered in Chapter 1.

<!-- /chunk -->

<!-- chunk:key-takeaways -->

## Key Takeaways

- **LLMs enable cognitive planning** by translating abstract natural language goals into ordered action sequences, leveraging commonsense reasoning and generalization.
- **Three types of context** are essential: environmental state (from Isaac ROS), robot capabilities (ROS 2 action servers), and task history (execution outcomes).
- **Prompting patterns** significantly impact planning quality:
  - **Chain-of-Thought** improves task decomposition through explicit reasoning
  - **Few-shot learning** grounds LLMs in robot-specific action vocabulary
  - **ReAct** enables adaptive replanning based on execution feedback
- **Scripted vs. adaptive planning** trade-offs: scripted execution offers low latency and determinism; LLM planning provides flexibility and handles novel commands.
- **Hybrid architectures** combine LLM high-level coordination with scripted low-level primitives, balancing adaptability and reliability.
- **LLMs coordinate primitives** from Nav2 (navigation), Isaac ROS (perception), and manipulation controllers (Module 1) by respecting preconditions, sequencing constraints, and integrating execution feedback.

<!-- /chunk -->

## Next Steps

We've explored VLA foundations (Chapter 1), voice-to-action pipelines (Chapter 2), and LLM cognitive planning (Chapter 3). **Chapter 4** integrates all components into a **complete autonomous humanoid system**, demonstrating end-to-end execution with a detailed capstone example: "Go to the kitchen and bring me a cup."

---

## Code Example: LLM Prompt Structure for Robot Planning

```python
# Conceptual LLM Prompt Structure for Robot Task Planning
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://platform.openai.com/docs/

def create_robot_planning_prompt(user_command, environmental_context, robot_capabilities):
    """
    Constructs an LLM prompt for robot task planning with context and few-shot examples.

    Args:
        user_command: Natural language goal from user
        environmental_context: Dict with robot location, visible objects, obstacles
        robot_capabilities: Dict with available actions and preconditions
    """
    system_prompt = """You are a robot task planner. Given a natural language command,
    decompose it into a sequence of executable robot actions.

    Available actions: navigate(location), detect_object(object_name), grasp(object_id),
                       place(location), hand_over()

    Think step-by-step and ensure preconditions are met before each action."""

    # Environmental context from Isaac ROS perception (Module 3)
    context_text = f"""
Current Robot State:
- Location: {environmental_context['location']}
- Visible Objects: {environmental_context['objects']}
- Available Actions: {robot_capabilities['actions']}
- Physical Constraints: {robot_capabilities['constraints']}

Task History:
{environmental_context.get('history', 'None')}
"""

    # Few-shot examples (in-context learning)
    examples = """
Example 1:
Command: "Pick up the red block"
Reasoning: First navigate to the block, then grasp it
Actions:
1. navigate_to_object(red_block)
2. grasp(red_block)

Example 2:
Command: "Go to the kitchen"
Reasoning: Simple navigation task
Actions:
1. navigate(kitchen)

Example 3:
Command: "Find the cup and bring it here"
Reasoning: Must detect cup location first, then approach, grasp, and return
Actions:
1. detect_object(cup)
2. navigate_to_object(cup)
3. grasp(cup)
4. navigate(user_location)
5. hand_over(cup)
"""

    # Construct complete prompt with Chain-of-Thought instruction
    full_prompt = f"""{system_prompt}

{context_text}

{examples}

Now plan for the following command. Think step-by-step about preconditions and sequencing.

Command: "{user_command}"

Reasoning:"""

    return full_prompt


# Example usage
environmental_context = {
    'location': 'living_room',
    'objects': ['cup (x=2.3, y=1.1)', 'table (x=2.0, y=1.0)'],
    'history': 'navigate(kitchen) - SUCCESS'
}

robot_capabilities = {
    'actions': ['navigate', 'detect_object', 'grasp', 'place', 'hand_over'],
    'constraints': 'reach_radius: 0.8m, payload: 2kg max'
}

prompt = create_robot_planning_prompt(
    user_command="Go to the kitchen and bring me a cup",
    environmental_context=environmental_context,
    robot_capabilities=robot_capabilities
)

# Send prompt to LLM API (e.g., OpenAI GPT-4, Anthropic Claude)
# llm_response = llm_client.complete(prompt)
# Parse LLM response to extract action sequence
# Execute actions via ROS 2 action servers (Action Mapper from Chapter 2)
```

This prompt structure demonstrates how to integrate environmental context (from Isaac ROS), robot capabilities (from ROS 2 action servers), and few-shot examples to guide LLM planning. The CoT instruction ("Think step-by-step") improves reasoning quality, and task history enables replanning based on prior execution outcomes.
