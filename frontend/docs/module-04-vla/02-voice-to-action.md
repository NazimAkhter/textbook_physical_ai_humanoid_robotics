---
id: voice-to-action
title: "Voice-to-Action Pipelines"
sidebar_label: "Voice-to-Action Pipelines"
sidebar_position: 2
description: "Learn how voice-to-action pipelines transform speech commands into robot execution through speech recognition, natural language understanding, and ROS 2 action mapping with real-time constraints."
keywords: ["voice control", "speech recognition", "NLU", "intent extraction", "action mapping", "ROS 2 actions", "real-time robotics", "Whisper", "STT"]
learning_objectives:
  - "Describe the voice-to-action pipeline from speech input to robot execution"
  - "Explain the role of speech recognition, intent extraction, and ROS 2 action mapping"
  - "Identify challenges in real-time speech processing for robotics contexts"
---

# Voice-to-Action Pipelines

<!-- chunk:introduction -->

## Introduction

Chapter 1 introduced VLA as the cognitive layer enabling humanoid robots to understand language and reason about tasks. Now we dive into **voice-to-action pipelines**—the interface that translates spoken commands into physical robot execution.

Voice control offers significant advantages for human-robot interaction:
- **Hands-free operation**: Users can command robots while performing other tasks
- **Accessibility**: Natural language is more intuitive than programming or graphical interfaces
- **Flexibility**: Voice commands adapt to diverse user phrasings without rigid syntax

This chapter explores how voice-to-action pipelines work, covering four key stages: **speech recognition** (audio → text), **natural language understanding** (text → intent), **intent extraction** (identifying goals), and **action mapping** (intent → ROS 2 actions). We'll examine real-time constraints that shape system design and integration with the ROS 2 ecosystem established in Module 1.

<!-- /chunk -->

<!-- chunk:speech-recognition -->

## Speech Recognition for Robotics

**Speech recognition** converts audio streams from microphones into text transcriptions that downstream systems can process. For robotics applications, we adopt a **service abstraction pattern**—defining a technology-agnostic interface rather than coupling to a specific vendor.

### Service Abstraction Pattern

The speech recognition component exposes a standard interface (e.g., ROS 2 service) that accepts audio input and returns text output. This abstraction allows swapping implementations without changing the rest of the pipeline:

- **OpenAI Whisper**: Open-source, state-of-the-art accuracy, multilingual support (~500ms latency on GPU)
- **Google Cloud Speech-to-Text**: Streaming recognition, low latency (&lt;300ms), robust noise handling (requires cloud connectivity)
- **Azure Speech Services**: Real-time streaming, speaker recognition, custom vocabulary (~200-400ms)

**Whisper as Concrete Example**: Whisper demonstrates feasibility for on-device robotics STT. Its transformer-based architecture achieves high accuracy across accents and noisy environments, making it suitable for home and industrial settings.

### Key Characteristics for Robotics

1. **Latency**: Target &lt;500ms from audio capture to text output. This constraint drives architectural choices—cloud STT offers accuracy but adds network latency; on-device models like Whisper provide faster response but require GPU acceleration.

2. **Robustness**: Robots operate in noisy environments (factory machinery, household appliances). Speech recognition must handle:
   - Background noise filtering
   - Accent and dialect variations
   - Domain-specific vocabulary (e.g., "navigate," "grasp," "Isaac ROS")

3. **Accuracy**: Misrecognized commands lead to incorrect actions. Domain-specific fine-tuning improves recognition of robotics terminology.

### ROS 2 Integration

Speech recognition integrates as a **ROS 2 service node**:

```
Audio Topic (sensor_msgs/Audio) → Speech Recognition Node → Text Service Response
```

This allows any node in the system to request transcription, maintaining modularity.

```
Audio Input (microphone)
    ↓
Speech Recognition Service (&lt;500ms)
    ↓
Text Output ("Go to the kitchen")
```

<!-- /chunk -->

<!-- chunk:nlu -->

## Natural Language Understanding

**Natural Language Understanding (NLU)** transforms raw text into structured representations that robots can act upon. While speech recognition handles audio-to-text conversion, NLU extracts **meaning** from text.

### NLU Tasks for Robotics

1. **Intent Classification**: Identify the user's goal
   - Examples: `navigate`, `grasp`, `detect`, `search`, `follow`

2. **Entity Recognition**: Extract relevant objects, locations, and attributes
   - "Pick up the **red cup** on the **table**" → entities: `object=cup`, `color=red`, `location=table`

3. **Parameter Extraction**: Determine action-specific details
   - "Move **forward 2 meters**" → `direction=forward`, `distance=2.0`

### Example Transformation

**Input**: "Pick up the red cup on the table"

**NLU Output**:
```json
{
  "intent": "grasp",
  "object": "cup",
  "color": "red",
  "location": "table"
}
```

This structured intent feeds into the Action Mapper (next section), which translates it to a ROS 2 action goal.

### Lightweight Models for Low Latency

Robotics NLU must process commands in &lt;100ms. This constraint favors **lightweight models** (e.g., distilled BERT, ONNX-optimized transformers) over large language models. For simple commands ("go to kitchen," "stop"), rule-based intent extractors or small neural classifiers suffice.

**Distinction from LLM Planning**: NLU extracts intent from single commands, while LLM-based cognitive planning (Chapter 3) decomposes complex multi-step goals. "Bring me a cup" requires LLM decomposition into navigate → detect → grasp → return, whereas "go to kitchen" maps directly to a `NavigateToPose` action via NLU.

<!-- /chunk -->

<!-- chunk:action-mapping -->

## Intent to Action Mapping

The **Action Mapper** translates structured intents into **ROS 2 action goals** that Nav2, Isaac ROS, and manipulation controllers can execute.

### ROS 2 Action Server Recap

From Module 1, recall the ROS 2 action interface:
- **Goal**: Desired outcome (e.g., target pose for navigation)
- **Feedback**: Progress updates during execution (e.g., distance remaining)
- **Result**: Success/failure status and final outcome

### Mapping Examples

| Intent | ROS 2 Action | Action Server | Module |
|--------|-------------|---------------|--------|
| `{intent: "navigate", target: "kitchen"}` | `NavigateToPose(kitchen_pose)` | Nav2 | Module 3 |
| `{intent: "grasp", object: "cup"}` | `Grasp(target_pose)` | Manipulation | Module 1 |
| `{intent: "detect", object: "cup"}` | `DetectObjects(class_id="cup")` | Isaac ROS | Module 3 |

The Action Mapper maintains a **lookup table** or **semantic mapping** that associates intents with action types. For `navigate` intents, it queries a location database (e.g., "kitchen" → `(x=5.0, y=3.0, z=0.0)` in the map frame) and constructs a `NavigateToPose` goal.

### Validation and Execution

Before sending goals, the Action Mapper validates:
- **Feasibility**: Is the target location within the map? Is the object within reach?
- **Preconditions**: Does grasping require prior object detection?
- **Parameter constraints**: Are velocity limits, joint angles, and safety thresholds respected?

If validation passes, the mapper sends the goal to the appropriate ROS 2 action server and monitors feedback. Success/failure results inform replanning (covered in Chapter 3).

**Integration with ROS 2 Ecosystem**: The Action Mapper leverages ROS 2 actions (Module 1), Nav2 path planning (Module 3), and Isaac ROS perception (Module 3), demonstrating the composability of the VLA stack.

<!-- /chunk -->

<!-- chunk:real-time-constraints -->

## Real-Time Constraints

Voice-controlled robotics demands **low-latency processing** to ensure responsive interaction. Users expect robots to react quickly—delays exceeding 4-5 seconds feel sluggish and break the sense of natural interaction.

### Latency Budget Breakdown

| Component | Target Latency | Purpose |
|-----------|----------------|---------|
| **Speech Recognition** | &lt;500ms | Audio stream → text transcription |
| **NLU / Intent Extraction** | &lt;100ms | Text → structured intent |
| **LLM Planning** | 1-3s | High-level task decomposition (Chapter 3) |
| **Total (Command → Action)** | &lt;4s | Acceptable user experience threshold |

**Trade-off Analysis**:
- **Accuracy vs. Latency**: Cloud-based STT (Google, Azure) achieves higher accuracy but adds network latency (~200-500ms round-trip). On-device models (Whisper) reduce latency but require local GPU compute.
- **Flexibility vs. Predictability**: LLM planning (1-3s) enables complex reasoning but adds delay. For simple commands ("stop," "go forward"), scripted intent mapping (&lt;10ms) suffices.

### Robustness Challenges

1. **Background Noise**: Industrial robots operate near machinery; home robots contend with appliances, conversations, and media. Robust STT requires noise cancellation and multi-microphone arrays for beamforming.

2. **Accent and Dialect Variations**: Multilingual support and accent-adaptive models (Whisper excels here) improve accessibility.

3. **Ambiguous Commands**: "Go there" requires clarification—where is "there"? Systems must detect ambiguity and prompt users ("Which location: kitchen, living room, or office?").

4. **Continuous Listening vs. Wake-Word Activation**: Always-on listening improves responsiveness but drains battery and raises privacy concerns. Wake-word detection ("Hey Robot") balances usability and power efficiency.

### Design Implications

Real-time constraints shape architectural choices:
- **Pipeline parallelism**: Start NLU processing before full transcription completes (streaming STT)
- **Local inference**: Deploy lightweight models on robot hardware to minimize network dependency
- **Graceful degradation**: If LLM planning exceeds latency budget, fall back to scripted command execution

These trade-offs recur throughout VLA system design, balancing user experience, computational cost, and system reliability.

<!-- /chunk -->

<!-- chunk:pipeline-architecture -->

## Voice-to-Action Pipeline Architecture

The complete voice-to-action pipeline integrates all components discussed, showing data flow from user speech to physical robot execution:

```
User Voice Command
    ↓
┌─────────────────────────────────┐
│ Speech Recognition (&lt;500ms)     │ → Audio stream → Text
│ (Whisper, Google STT, Azure)    │
└─────────────┬───────────────────┘
              ↓ "Go to kitchen"
┌─────────────────────────────────┐
│ NLU / Intent Extraction (&lt;100ms)│ → Text → Structured intent
│ (BERT, rule-based classifier)   │
└─────────────┬───────────────────┘
              ↓ {command: "navigate", target: "kitchen"}
┌─────────────────────────────────┐
│ Action Mapper                   │ → Intent → ROS 2 action goal
│ (Semantic mapping + validation) │
└─────────────┬───────────────────┘
              ↓ NavigateToPose(kitchen_pose)
┌─────────────────────────────────┐
│ ROS 2 Action Server (Nav2)      │ → Action goal → Physical execution
│ (Path planning + obstacle avoid)│
└─────────────┬───────────────────┘
              ↓
         Robot Motion
              ↓
       Execution Result
    (Success: arrived at kitchen
     Failure: path blocked)
              ↓
       ┌─────────────┐
       │ Feedback    │ ← Informs replanning (Chapter 3)
       └─────────────┘
```

**Key Integration Points**:
- **ROS 2 Topics**: Audio streams publish to `sensor_msgs/Audio` topics
- **ROS 2 Services**: Speech recognition and NLU expose service interfaces
- **ROS 2 Actions**: Action Mapper clients connect to Nav2, Isaac ROS, and manipulation action servers
- **Feedback Loop**: Execution results (success/failure, perception updates) flow back to inform cognitive planning

This architecture maintains **modularity**—each component can be developed, tested, and replaced independently—while ensuring **end-to-end integration** through standardized ROS 2 interfaces.

<!-- /chunk -->

<!-- chunk:key-takeaways -->

## Key Takeaways

- **Voice-to-action pipelines** transform speech to robot execution through four stages: speech recognition, NLU, action mapping, and execution.
- **Speech recognition** (audio → text) must achieve &lt;500ms latency and handle noisy environments; service abstraction enables technology flexibility (Whisper, Google STT, Azure).
- **NLU** (text → structured intent) extracts goals, entities, and parameters in &lt;100ms using lightweight models.
- **Action Mapper** translates intents to ROS 2 action goals, integrating with Nav2 (Module 3) for navigation and Isaac ROS (Module 3) for perception.
- **Real-time constraints** (&lt;4s command-to-action) drive trade-offs between accuracy, latency, and computational cost.
- **Robustness challenges**—noise, accents, ambiguity—require careful system design with fallback mechanisms.

<!-- /chunk -->

## Next Steps

Voice-to-action pipelines handle simple commands well ("go to kitchen," "stop"). **Chapter 3** explores **LLM-based cognitive planning** for complex multi-step tasks requiring reasoning, context integration, and adaptive replanning.

---

## Code Example: ROS 2 Action Client for Voice-Controlled Navigation

```python
# Conceptual ROS 2 Action Client for Voice-Controlled Navigation
# Shows integration pattern - NOT executable without full system setup
# For implementation details, see: https://docs.ros.org/en/humble/Tutorials/

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class VoiceControlledNavigator(Node):
    def __init__(self):
        super().__init__('voice_navigator')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def execute_voice_command(self, intent):
        """
        Translates voice intent to ROS 2 action goal

        Args:
            intent: Dictionary with command and target location
                    e.g., {command: "navigate", target: "kitchen"}
        """
        if intent['command'] == 'navigate':
            target_location = intent['target']

            # Create ROS 2 action goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.get_location_pose(target_location)
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            # Send goal to Nav2 action server
            self.nav_client.wait_for_server()
            self.get_logger().info(f'Navigating to {target_location}...')

            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Monitor navigation progress"""
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance_remaining:.2f}m')

    def goal_response_callback(self, future):
        """Handle action server response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Process navigation result"""
        result = future.result().result
        if result:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().warn('Navigation failed - replanning required')

    def get_location_pose(self, location_name):
        """Lookup location coordinates from semantic map"""
        # In practice, query location database or semantic map
        location_db = {
            'kitchen': PoseStamped(pose={'position': {'x': 5.0, 'y': 3.0, 'z': 0.0}}),
            'living_room': PoseStamped(pose={'position': {'x': 1.0, 'y': 1.0, 'z': 0.0}})
        }
        return location_db.get(location_name)

def main(args=None):
    rclpy.init(args=args)
    navigator = VoiceControlledNavigator()

    # Example: Process voice intent from NLU pipeline
    intent = {'command': 'navigate', 'target': 'kitchen'}
    navigator.execute_voice_command(intent)

    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()
```

This example demonstrates the Action Mapper component translating voice intents to ROS 2 `NavigateToPose` actions. The feedback and result callbacks enable monitoring execution and triggering replanning on failure—key capabilities for adaptive VLA systems.
