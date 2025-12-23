# SVG Diagram Specifications

**Purpose**: Specifications and guidelines for creating 4 SVG diagrams for introduction chapter
**Date**: 2025-12-23
**Feature**: 001-intro-physical-ai-humanoid

## Infima Theme Color Palette

**Primary Colors** (from Docusaurus default theme):
- Primary: `#2e8555` (Green)
- Primary Dark: `#29784c`
- Primary Darker: `#277148`
- Primary Darkest: `#205d3b`
- Primary Light: `#33925d`
- Primary Lighter: `#359962`
- Primary Lightest: `#3cad6e`

**Secondary Colors**:
- Secondary: `#25c2a0` (Teal/Cyan)
- Secondary Dark: `#21af90`
- Secondary Darker: `#1fa588`
- Secondary Darkest: `#1a8870`
- Secondary Light: `#29d5b0`
- Secondary Lighter: `#32d8b4`
- Secondary Lightest: `#4fddbf`

**Neutral Colors**:
- Text: `#1c1e21` (Almost black)
- Background: `#ffffff` (White)
- Border: `#dadde1` (Light gray)
- Code Background: `#f6f7f8`

**Accessibility Requirements**:
- Contrast ratio minimum: 4.5:1 for normal text
- Contrast ratio minimum: 3:1 for large text and graphics
- All colors tested against white background

---

## Diagram 1: AI Paradigm Comparison

**File**: `frontend/static/img/introduction/comparative-diagram.svg`
**Dimensions**: 800px × 500px
**Purpose**: Side-by-side comparison of Traditional AI vs Physical AI

### Content Structure:

**Left Side - Traditional AI**:
- Input: Data (text, images, structured data)
- Processing: Neural networks in cloud/servers
- Output: Predictions, classifications, recommendations
- Examples: Image recognition, language translation, recommendation systems
- Key Characteristic: Virtual, data-centric

**Right Side - Physical AI**:
- Input: Sensor data (cameras, LIDAR, force/torque, proprioception)
- Processing: Embodied reasoning + real-time control
- Output: Physical actions (movement, manipulation, interaction)
- Examples: Warehouse robots, surgical assistants, autonomous vehicles
- Key Characteristic: Embodied, action-centric

**Visual Elements**:
- Two columns with clear separation
- Icons for each type (brain/chip for traditional, robot for physical)
- Arrows showing data flow
- Color coding: Primary green for Physical AI, Secondary teal for Traditional AI

### Alt Text:
"Comparative diagram showing Traditional AI on the left (data input, neural network processing, prediction output) versus Physical AI on the right (sensor input, embodied reasoning, physical action output). Traditional AI operates in virtual space processing data to make predictions. Physical AI integrates sensors, computation, and actuators to interact with the physical world through perception, decision-making, and action."

---

## Diagram 2: Physical AI Concept Map

**File**: `frontend/static/img/introduction/physical-ai-concept-map.svg`
**Dimensions**: 800px × 600px
**Purpose**: Show relationships between Physical AI, Traditional AI, Robotics, and Embodied Intelligence

### Content Structure:

**Central Node**: Physical AI (large circle, primary green)

**Connected Concepts** (with relationship labels):
- Traditional AI (above left): "Builds upon" → AI/ML algorithms, neural networks
- Robotics (above right): "Integrates with" → Sensors, actuators, control systems
- Embodied Intelligence (below): "Implements" → Physical interaction, sensorimotor coordination
- Computer Vision (left): "Uses" → Perception, object recognition
- Motion Planning (right): "Requires" → Path planning, collision avoidance
- Human-Robot Interaction (bottom): "Enables" → Safe collaboration, natural interfaces

**Visual Elements**:
- Node-and-edge graph layout
- Color gradient from secondary (traditional AI) to primary (Physical AI)
- Directional arrows with relationship labels
- Size indicates importance/centrality

### Alt Text:
"Concept map centered on Physical AI showing its relationships to related fields. Physical AI builds upon Traditional AI (machine learning and neural networks), integrates with Robotics (sensors, actuators, control), and implements Embodied Intelligence (physical interaction and sensorimotor coordination). Connected fields include Computer Vision for perception, Motion Planning for navigation, and Human-Robot Interaction for safe collaboration. Arrows indicate directional relationships with labels like 'builds upon,' 'integrates with,' and 'enables.'"

---

## Diagram 3: System Architecture Diagram

**File**: `frontend/static/img/introduction/architecture-diagram.svg`
**Dimensions**: 800px × 600px
**Purpose**: Show how ROS 2, Digital Twins, NVIDIA Isaac, and VLA components interact

### Content Structure:

**Layer 1 - Bottom (Foundation)**:
- ROS 2 Middleware
  - Nodes, Topics, Services
  - DDS Communication
  - Real-time capabilities
- Color: Primary green base

**Layer 2 - Middle (Simulation & Training)**:
- Digital Twin (left): Virtual replica, testing, validation
- NVIDIA Isaac (right): Isaac Sim, Isaac Gym, physics engine
- Bidirectional arrows to ROS 2 layer
- Color: Secondary teal

**Layer 3 - Top (Intelligence)**:
- Vision-Language-Action (VLA) System
  - Vision input, Language understanding, Action output
  - Arrows down to both Digital Twin and NVIDIA Isaac
- Color: Primary light green

**Horizontal Layer (Right Side)**:
- Real-World Robot (receives commands from all layers)
- Sensor feedback loop back to VLA

**Data Flow Arrows**:
- Solid lines: Primary data flow
- Dashed lines: Feedback loops
- Color-coded by layer

### Alt Text:
"Layered system architecture diagram showing the Physical AI technology stack. Bottom layer: ROS 2 middleware providing communication infrastructure with nodes, topics, and services. Middle layer: Digital Twin for virtual testing alongside NVIDIA Isaac for simulation and physics. Top layer: Vision-Language-Action (VLA) system processing visual input and language to generate robot actions. Arrows show data flow from sensors through processing layers to actuators, with feedback loops. A real-world robot on the right receives commands from all layers and sends sensor data back."

---

## Diagram 4: Learning Path Flowchart

**File**: `frontend/static/img/introduction/learning-path-flowchart.svg`
**Dimensions**: 600px × 800px (vertical orientation)
**Purpose**: Visualize module progression from Introduction through Capstone

### Content Structure:

**Vertical Flow** (top to bottom):

1. **Introduction** (you are here)
   - Box: Rounded rectangle, primary green
   - Icon: Book/welcome symbol

2. **Module 1: ROS 2 as Robotic Nervous System**
   - Box: Rectangle, primary color
   - Key topics: Nodes, topics, pub/sub

3. **Module 2: Digital Twins & Simulation**
   - Box: Rectangle, primary color
   - Key topics: Virtual replicas, testing

4. **Module 3: NVIDIA Isaac & Physical AI**
   - Box: Rectangle, primary color
   - Key topics: Isaac Sim, GPU acceleration

5. **Module 4: Vision-Language-Action Systems**
   - Box: Rectangle, primary color
   - Key topics: VLA models, multimodal learning

6. **Capstone Project: Autonomous Humanoid**
   - Box: Rounded rectangle, secondary teal
   - Icon: Trophy/completion symbol

**Connection Lines**:
- Solid arrows: Recommended linear path
- Dashed arrows: Optional skip/review paths (e.g., "Skip M1 if familiar with ROS 2")

**Side Annotations**:
- Estimated time per module
- Prerequisites noted
- Dependencies highlighted

### Alt Text:
"Vertical flowchart showing the book's learning path. Starting from Introduction at the top, the path flows through four modules in sequence: Module 1 (ROS 2 as Robotic Nervous System), Module 2 (Digital Twins & Simulation), Module 3 (NVIDIA Isaac & Physical AI), and Module 4 (Vision-Language-Action Systems). The path concludes with a Capstone Project combining all modules. Solid arrows indicate the recommended linear progression, while dashed arrows show optional paths for readers with existing knowledge. Each module box displays key topics and estimated completion time."

---

## Tool Recommendations

### Option 1: Figma (Recommended)
- **Pros**: Professional, web-based, exports clean SVG, collaboration features
- **Cons**: Requires account
- **Color Setup**: Create color palette with hex values above
- **Export**: File → Export → SVG (ensure "Include id attribute" checked)

### Option 2: draw.io (diagrams.net)
- **Pros**: Free, no account needed, web or desktop, good for technical diagrams
- **Cons**: Less polished than Figma
- **Color Setup**: Custom palette in Format panel
- **Export**: File → Export as → SVG

### Option 3: Inkscape
- **Pros**: Free, open-source, powerful vector editing
- **Cons**: Steeper learning curve, desktop only
- **Color Setup**: Edit → Preferences → Interface → Theme
- **Export**: File → Save As → Optimized SVG

### Option 4: Excalidraw
- **Pros**: Simple, sketchy style, web-based, free
- **Cons**: More informal aesthetic (may not fit professional tone)
- **Color Setup**: Built-in palette customization
- **Export**: Export as SVG

---

## SVG Optimization Checklist

Before adding to repository:

- [ ] Remove unnecessary metadata and comments
- [ ] Optimize file size (use SVGO or similar)
- [ ] Ensure text is converted to paths or uses web-safe fonts
- [ ] Verify colors match Infima palette exactly
- [ ] Test at different viewport sizes (responsive)
- [ ] Validate accessibility: proper `<title>` and `<desc>` tags in SVG
- [ ] Confirm contrast ratios meet WCAG 2.1 AA (4.5:1 minimum)
- [ ] Add `role="img"` and `aria-labelledby` attributes if needed

---

## SVG Template Structure

```xml
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 600" role="img" aria-labelledby="diagram-title">
  <title id="diagram-title">Diagram Title</title>
  <desc>Detailed description for screen readers (50-100 words)</desc>

  <!-- Define color palette -->
  <defs>
    <style>
      .primary { fill: #2e8555; }
      .secondary { fill: #25c2a0; }
      .text { fill: #1c1e21; }
      .border { stroke: #dadde1; }
    </style>
  </defs>

  <!-- Diagram content -->
  <g id="content">
    <!-- Shapes, text, paths here -->
  </g>
</svg>
```

---

## Testing and Validation

### Contrast Testing Tools:
- WebAIM Contrast Checker: https://webaim.org/resources/contrastchecker/
- Colorable: https://colorable.jxnblk.com/

### SVG Validation:
- W3C Validator: https://validator.w3.org/
- SVG Sanitizer: https://svg-sanitizer.netlify.app/

### Accessibility Testing:
- axe DevTools (browser extension)
- NVDA or JAWS screen reader testing

---

## Notes

- All diagrams must work in both light and dark mode (verify Docusaurus theme switching)
- Mobile responsiveness: diagrams should scale gracefully on small screens
- Consider adding `loading="lazy"` attribute in markdown for performance
- File size target: < 50KB per diagram (optimized)
- Use semantic grouping (`<g>` tags) for better organization

**Status**: Specifications complete, ready for diagram creation in Phase 3+
