# Chapter Contract: Ch2 - Isaac Sim for Photorealistic Simulation

**File**: `frontend/docs/module-03-nvidia-isaac/02-isaac-sim.md`
**Word Target**: 1500-2500 words
**Priority**: P2

## Frontmatter

```yaml
id: isaac-sim
title: "Isaac Sim: Photorealistic Simulation and Synthetic Data"
sidebar_label: "Isaac Sim"
sidebar_position: 2
description: "Learn how Isaac Sim creates photorealistic simulations and generates synthetic training data for perception models."
keywords: ["isaac sim", "omniverse", "synthetic data", "domain randomization", "sim-to-real", "photorealistic rendering", "usd"]
learning_objectives:
  - "Explain why photorealistic rendering matters for perception model training"
  - "Describe three techniques for closing the sim-to-real gap"
  - "Identify how synthetic data generation automates perception dataset creation"
```

## Required Sections

### 1. Introduction (~200 words)
- Connect to Module 2 Digital Twins—Isaac Sim takes simulation further
- Problem: Standard simulations don't look real enough for vision training
- Promise: Photorealistic rendering + automatic labeling

### 2. Omniverse and USD Foundation (~400 words)
- **Omniverse**: NVIDIA's collaboration platform for 3D
- **USD (Universal Scene Description)**: Pixar's scene format
- Why USD matters: composable, versionable, collaborative
- Not just graphics—physics-enabled simulation

### 3. Physically Based Rendering (~400 words)
- **FR-003**: MUST explain PBR and sensor realism
- What makes rendering "physically based"
- Materials: albedo, roughness, metallic, normal maps
- RTX ray tracing for accurate lighting, reflections, shadows
- Why this matters: cameras see what humans see

### 4. Synthetic Data Generation (~400 words)
- **FR-004**: MUST describe synthetic data concepts
- Omniverse Replicator for programmatic scene generation
- Automatic ground truth: segmentation masks, bounding boxes, depth
- Scale: thousands of labeled images per hour
- Comparison to manual annotation cost/time

### 5. Closing the Sim-to-Real Gap (~500 words)
- **FR-005**: MUST explain at least two techniques
- Technique 1: **Domain Randomization**
  - Vary lighting, textures, object positions
  - Train models robust to real-world variation
- Technique 2: **Photorealistic Fidelity**
  - Match real-world appearance
  - Reduce distribution shift
- Technique 3: **Accurate Sensor Models**
  - Add noise, distortion, latency
  - Match real camera characteristics
- When does sim-to-real gap still exist?

### 6. Key Takeaways
- Isaac Sim = Omniverse + USD + PhysX for robotics
- PBR creates images that match real-world appearance
- Synthetic data provides automatic, scalable labeling
- Domain randomization helps models generalize
- Photorealism reduces but doesn't eliminate sim-to-real gap

### 7. Next Steps
- Transition to Chapter 3: Isaac ROS for using trained models in real-time perception

## Requirements Coverage

| Requirement | Section | How Addressed |
|-------------|---------|---------------|
| FR-003 | Section 3 | PBR and sensor realism explained |
| FR-004 | Section 4 | Synthetic data generation concepts |
| FR-005 | Section 5 | Domain randomization + photorealism (2+ techniques) |
| FR-012 | All | Word count 1500-2500 |
| FR-013 | All | Technical, non-marketing tone |
| FR-014 | Snippet | Conceptual USD/Replicator example |

## Acceptance Scenarios

1. **Given** Isaac Sim rendering explanation, **When** reader completes chapter, **Then** they can explain why photorealism matters for perception
2. **Given** synthetic data concepts, **When** asked about domain randomization, **Then** reader can explain how it helps generalization
3. **Given** sim-to-real discussion, **When** reader encounters perception failure, **Then** they can identify simulation fidelity issues

## Conceptual Code Snippet

```python
# Conceptual Omniverse Replicator workflow
# Illustrates synthetic data generation concepts
# NOT executable without full Omniverse installation

import omni.replicator.core as rep

def generate_training_data():
    # Create randomized lighting for domain randomization
    with rep.new_layer():
        # Dome light with random rotation
        light = rep.create.light(
            light_type="dome",
            rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0))
        )

        # Load robot model (references URDF from Module 1)
        robot = rep.create.from_usd("/robots/humanoid.usd")

        # Randomize environment textures
        floor = rep.get.prim_at_path("/World/Floor")
        with floor:
            rep.randomizer.texture(
                textures=["wood_01", "concrete_02", "tile_03"]
            )

        # Configure automatic ground truth
        with rep.trigger.on_frame():
            rep.annotators.write(
                annotators=["rgb", "semantic_segmentation", "depth"],
                output_dir="/synthetic_dataset/"
            )

    # Generate 1000 randomized frames
    rep.orchestrator.run(num_frames=1000)
```

## Diagram Description

:::info Diagram: Synthetic Data Generation Pipeline

**Type**: flow

**Description**: Shows the pipeline from Isaac Sim scene to trained perception model.

**Key Elements**:
- Isaac Sim Environment (USD scene, robot, objects)
- Replicator (domain randomization, camera placement)
- Rendered Output (RGB images)
- Ground Truth (segmentation, depth, bounding boxes)
- Training Pipeline (to perception model)
- Real-World Deployment (model inference on actual camera)
- Sim-to-Real arrow showing gap

:::

## Chunk Map

- `chunk:isaac-sim-overview` - What Isaac Sim provides
- `chunk:omniverse-usd` - Platform and scene format
- `chunk:pbr-rendering` - Physically based rendering explained
- `chunk:synthetic-data-generation` - Automatic labeling pipeline
- `chunk:sim-to-real-gap` - Techniques for closing the gap
