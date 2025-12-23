# Implementation Plan: Module 2 – The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin-gazebo-unity` | **Date**: 2024-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-digital-twin-gazebo-unity/spec.md`

## Summary

Create educational content for Module 2 covering digital twins, physics simulation with Gazebo, visualization with Unity, and sensor simulation for humanoid robotics. The module includes 4 chapters (2,500–4,000 words total) in Markdown format for Docusaurus, with illustrative code snippets, diagram descriptions, and key takeaways per chapter. Content builds on Module 1 (ROS 2, URDF) and serves as single source of truth for both the book and future RAG chatbot integration.

## Technical Context

**Language/Version**: Markdown (CommonMark + MDX), Docusaurus 3.x
**Primary Dependencies**: Docusaurus Classic Theme, docs plugin
**Storage**: File-based (Git repository)
**Testing**: Manual review, Docusaurus build validation, word count checks
**Target Platform**: Web (Vercel deployment), mobile-responsive
**Project Type**: Documentation/Educational content (web)
**Performance Goals**: Page load <3s, mobile-responsive rendering
**Constraints**: 2,500–4,000 words total, non-executable code snippets only
**Scale/Scope**: 4 chapters, ~15-25 minute reading time

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-Driven Development | ✅ PASS | Following `/sp.constitution` → `/sp.specify` → `/sp.clarify` → `/sp.plan` sequence |
| II. Educational Efficacy | ✅ PASS | Content connects simulation theory to humanoid robotics practice |
| III. Documentation–Agent Integration | ✅ PASS | Single-source content with RAG chunking markers |
| IV. Reproducibility by Default | ⚠️ PARTIAL | Code snippets illustrative only (per spec FR-003) |
| V. Technical Standards | ✅ PASS | Docusaurus/Vercel stack per constitution |
| VI. Constraints & Compliance | ✅ PASS | Claude Code + Spec-Kit Plus only |

**Principle IV Justification**: Module 2 spec explicitly requires "minimal illustrative snippets only" per FR-003 and out-of-scope declares "step-by-step simulation tutorials." Full executable examples would require simulation software installation, which is explicitly out-of-scope.

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin-gazebo-unity/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Research decisions and references
├── data-model.md        # Content entity definitions
├── quickstart.md        # Implementation guide
├── contracts/           # Structural contracts
│   └── content-structure.md
├── checklists/          # Validation checklists
│   └── requirements.md
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
frontend/docs/
└── module-02-digital-twin/
    ├── _category_.json           # Sidebar configuration
    ├── 01-digital-twins.md       # Chapter 1: Digital Twins in Robotics
    ├── 02-gazebo-physics.md      # Chapter 2: Physics Simulation with Gazebo
    ├── 03-unity-visualization.md # Chapter 3: Visualization and Interaction with Unity
    └── 04-sensor-simulation.md   # Chapter 4: Sensor Simulation for Perception
```

**Structure Decision**: Follows Module 1 pattern. Documentation site with docs plugin. Single module directory with numbered chapter files for predictable ordering and RAG-compatible paths. Module 2 placed after Module 1 in sidebar hierarchy.

## Architecture Sketch

### Content Authoring Flow

```
Spec-Kit Plus Workflow:
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  /sp.specify    │───▶│   /sp.plan      │───▶│   /sp.tasks     │
│  (spec.md)      │    │   (plan.md)     │    │   (tasks.md)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                      │
                                                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  /sp.implement  │◀───│  Content Files  │◀───│  Research +     │
│  (execution)    │    │  (Markdown)     │    │  Writing        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌─────────────────┐
                       │  Docusaurus     │
                       │  Build + Deploy │
                       └─────────────────┘
```

### Frontend Structure (Docusaurus)

```
Docusaurus Site:
┌────────────────────────────────────────────────────────┐
│  Header: Navigation, Search, Theme Toggle              │
├────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌────────────────────────────────┐  │
│  │  Sidebar     │  │  Main Content Area             │  │
│  │              │  │                                │  │
│  │  Module 1 ▸  │  │  [Chapter Title]               │  │
│  │  Module 2 ▾  │  │  [Content with headings,       │  │
│  │   Ch 1 ●     │  │   code snippets, diagrams]     │  │
│  │   Ch 2       │  │                                │  │
│  │   Ch 3       │  │  [Key Takeaways]               │  │
│  │   Ch 4       │  │                                │  │
│  │  Module 3 ▸  │  │  [Prev/Next Navigation]        │  │
│  │  Module 4 ▸  │  │                                │  │
│  │  Module 5 ▸  │  │                                │  │
│  └──────────────┘  └────────────────────────────────┘  │
├────────────────────────────────────────────────────────┤
│  [Future: RAG Chatbot Widget Slot]                     │
└────────────────────────────────────────────────────────┘
```

### Module 1 → Module 2 Content Flow

```
Module 1 Concepts (Prerequisites):          Module 2 Content (This Module):
┌─────────────────────────────────┐         ┌─────────────────────────────────┐
│  ROS 2 Middleware               │────────▶│  Digital Twin Concept           │
│  - Nodes, Topics, Services      │         │  - Why simulate before deploy   │
│  - Message passing              │         │  - Physics + sensors + control  │
└─────────────────────────────────┘         └─────────────────────────────────┘
┌─────────────────────────────────┐         ┌─────────────────────────────────┐
│  URDF Structure                 │────────▶│  Gazebo SDF Conversion          │
│  - Links, joints, transmissions │         │  - URDF → SDF pipeline          │
│  - Robot description format     │         │  - Model spawning in world      │
└─────────────────────────────────┘         └─────────────────────────────────┘
┌─────────────────────────────────┐         ┌─────────────────────────────────┐
│  rclpy Integration              │────────▶│  gz-ros2-control                │
│  - Publishers/Subscribers       │         │  - Control commands in sim      │
│  - Callbacks and executors      │         │  - Sensor data publishing       │
└─────────────────────────────────┘         └─────────────────────────────────┘
```

### RAG Integration Points

```
Content → RAG Pipeline:
┌─────────────────┐
│  Markdown Files │
│  (with chunk    │
│   markers)      │
└────────┬────────┘
         │ Build-time extraction
         ▼
┌─────────────────┐
│  Content        │
│  Processor      │
│  (future plugin)│
└────────┬────────┘
         │ Chunks + metadata
         ▼
┌─────────────────┐
│  Embedding      │
│  Generation     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Qdrant Cloud   │
│  Vector Storage │
└─────────────────┘
```

## Content Production Workflow

### Research-Concurrent Writing Approach

1. **Light Research** → Gather authoritative references (see research.md)
2. **Draft Writing** → Create initial chapter content
3. **Citation Integration** → Add references inline during writing (APA style per constitution)
4. **Refinement** → Iterative improvement based on review
5. **Validation** → Check against spec requirements

### Single-Source-of-Truth Enforcement

- All content in `frontend/docs/module-02-digital-twin/*.md`
- RAG chunking markers embedded in source files
- No separate RAG-specific content files
- Frontmatter keywords enable topic-based retrieval

## Phased Execution Plan

### Phase 1: Research (Current)

- [x] Identify Docusaurus configuration approach (reuse Module 1 pattern)
- [x] Define module/chapter structuring strategy
- [x] Establish Markdown conventions and frontmatter standards
- [x] Document navigation and sidebar approach
- [x] Define RAG content reuse strategy
- [x] Record decisions in research.md

### Phase 2: Foundation

- [ ] Create module directory `frontend/docs/module-02-digital-twin/`
- [ ] Add `_category_.json` for sidebar configuration
- [ ] Create chapter file stubs with frontmatter
- [ ] Verify sidebar integration with Module 1
- [ ] Verify Docusaurus build succeeds

### Phase 3: Content Development (Per Chapter)

**Chapter 1: Digital Twins in Robotics**
- [ ] Draft introduction and digital twin definition
- [ ] Write benefits section (safe testing, rapid iteration, cost reduction)
- [ ] Explain simulation fidelity levels and trade-offs
- [ ] Overview of simulation ecosystem (physics engines, renderers, sensors)
- [ ] Add diagram descriptions
- [ ] Write key takeaways
- [ ] Add RAG chunking markers
- [ ] Validate learning objectives coverage

**Chapter 2: Physics Simulation with Gazebo**
- [ ] Draft Gazebo architecture explanation
- [ ] Explain physics engines (ODE, Bullet, DART, Simbody)
- [ ] Write world modeling section (gravity, friction, contact dynamics)
- [ ] Explain URDF-to-SDF conversion (building on Module 1)
- [ ] Add ROS 2 integration patterns (gz-ros2-control)
- [ ] Include illustrative code snippets (SDF world, physics parameters)
- [ ] Add diagram descriptions
- [ ] Write key takeaways
- [ ] Add RAG chunking markers

**Chapter 3: Visualization and Interaction with Unity**
- [ ] Draft Unity for robotics visualization
- [ ] Explain high-fidelity rendering vs. physics accuracy trade-offs
- [ ] Write human-robot interaction scenarios
- [ ] Explain Unity Robotics Hub and ROS-TCP-Connector concepts
- [ ] Add Gazebo vs. Unity comparison table
- [ ] Include illustrative code snippets (Unity-ROS bridge concepts)
- [ ] Add diagram descriptions
- [ ] Write key takeaways
- [ ] Add RAG chunking markers

**Chapter 4: Sensor Simulation for Perception**
- [ ] Draft simulated sensors overview (cameras, LiDAR, IMU, force-torque)
- [ ] Explain noise models and calibration concepts
- [ ] Write synthetic data generation for perception training
- [ ] Explain sensor fusion in simulation
- [ ] Address sim-to-real gap for perception pipelines
- [ ] Include illustrative code snippets (sensor configuration concepts)
- [ ] Add diagram descriptions
- [ ] Write key takeaways
- [ ] Add RAG chunking markers

### Phase 4: Synthesis

- [ ] Cross-link chapters where concepts reference each other
- [ ] Cross-link to Module 1 (URDF, ROS 2 integration)
- [ ] Normalize terminology across all chapters
- [ ] Verify word count (2,500–4,000 total)
- [ ] Validate all code snippets have disclaimer comments
- [ ] Final review for scope compliance
- [ ] Run Docusaurus build and verify no errors

## Testing & Validation Strategy

### Structural Validation

| Check | Method | Pass Criteria |
|-------|--------|---------------|
| Module directory exists | File system check | `frontend/docs/module-02-digital-twin/` present |
| All chapters present | File count | 4 chapter files + `_category_.json` |
| Correct ordering | Frontmatter check | `sidebar_position` 1-4 |
| Sidebar links resolve | Docusaurus build | No broken link warnings |
| Build succeeds | `npm run build` | Exit code 0 |
| Module 1 cross-links work | Link validation | No broken references |

### Content Validation

| Check | Method | Pass Criteria |
|-------|--------|---------------|
| Word count | `wc -w` on all chapter files | 2,500–4,000 total |
| Learning objectives | Manual review | Each chapter covers stated objectives |
| Scope compliance | Manual review | No out-of-scope content |
| Code snippets valid | Syntax highlighting | No highlighting errors |
| Key takeaways present | Grep for "## Key Takeaways" | Present in all 4 chapters |
| Diagram descriptions | Grep for ":::info Diagram" | At least 1 per chapter |
| RAG chunks present | Grep for "<!-- chunk:" | Appropriate markers present |

### Spec Compliance

| Requirement | Validation |
|-------------|------------|
| FR-001: No installation required | Content review |
| FR-002: Builds on Module 1 URDF | Cross-references present |
| FR-003: Illustrative code with disclaimers | Disclaimer comments present |
| FR-004: RAG chunking markers | Chunk markers in all chapters |
| FR-005: Diagram descriptions | Admonition format used |
| FR-006: Gazebo vs Unity comparison | Comparison section present |
| FR-007: Sim-to-real gap explained | Content present without deployment procedures |
| FR-008: Sensor simulation coverage | Camera, LiDAR, IMU, force-torque covered |
| FR-009: ROS 2 integration patterns | Patterns referenced, no full tutorials |
| FR-010: Conceptual tone | Style review |
| FR-011: Word count 2,500-4,000 | Word count tool |
| FR-012: Chapter order | Directory listing + frontmatter |

## Decisions & Tradeoffs

### Decision 1: Module Directory Naming

**Options Considered**:
- A: `module-02-digital-twins-gazebo-unity/` (full descriptive)
- B: `module-02-digital-twin/` (concise, matches Module 1 pattern)
- C: `02-digital-twin/` (no "module" prefix)

**Selected**: Option B - `module-02-digital-twin/`

**Rationale**: Maintains consistency with Module 1 pattern (`module-01-ros2-nervous-system`), keeps paths readable, and avoids overly long directory names. The "module-" prefix provides clear categorization in the docs folder.

### Decision 2: Chapter File Naming

**Options Considered**:
- A: Descriptive slugs matching chapter titles
- B: Generic numbered files (01.md, 02.md)
- C: Topic-based slugs (digital-twins.md, gazebo.md)

**Selected**: Option A - Descriptive slugs

**Rationale**: Consistent with Module 1, enables RAG-friendly URLs, and provides context without opening files.

**File Mapping**:
- `01-digital-twins.md` → Chapter 1: Digital Twins in Robotics
- `02-gazebo-physics.md` → Chapter 2: Physics Simulation with Gazebo
- `03-unity-visualization.md` → Chapter 3: Visualization and Interaction with Unity
- `04-sensor-simulation.md` → Chapter 4: Sensor Simulation for Perception

### Decision 3: Gazebo vs Unity Balance

**Challenge**: Module covers both Gazebo and Unity, each with distinct use cases.

**Approach**:
- Gazebo (Chapter 2): Focus on physics simulation accuracy, ROS 2 integration
- Unity (Chapter 3): Focus on visualization, human-robot interaction
- Comparison table in Chapter 3 helps readers choose appropriate tool
- Avoid implying one is "better" - emphasize complementary strengths

**Tradeoff**: Dedicating a full chapter to each tool reduces depth but provides balanced coverage. Alternative of combining tools in one chapter would lose clarity on distinct use cases.

### Decision 4: Sim-to-Real Gap Treatment

**Constraint**: Spec requires explaining sim-to-real gap but excludes deployment procedures.

**Approach**:
- Explain gap conceptually in Chapter 1 (overview)
- Address physics gap in Chapter 2 (contact modeling limitations)
- Address visual gap in Chapter 3 (rendering vs. real perception)
- Address sensor gap in Chapter 4 (noise models, calibration)
- No deployment procedures or transfer learning techniques

**Rationale**: Distributed coverage reinforces the concept across contexts while staying within scope.

### Decision 5: Code Snippet Strategy

**Constraint**: Illustrative only, non-executable, with disclaimers.

**Approach**:
- Chapter 2: SDF world fragment, physics parameter configuration
- Chapter 3: Conceptual Unity-ROS bridge pseudocode
- Chapter 4: Sensor noise model configuration concept
- All snippets include: `<!-- NOTE: This is an illustrative example, not production code -->`

**Tradeoff**: Non-executable code limits hands-on learning but stays within spec scope. Executable tutorials would require installation guides (explicitly out-of-scope).

## Decisions Requiring ADR

The following architectural decisions were made during planning:

| Decision | Impact | ADR Needed? |
|----------|--------|-------------|
| Module naming convention | Content organization | No (follows established pattern) |
| Gazebo/Unity chapter split | Content structure | No (spec-defined structure) |
| Sim-to-real distributed coverage | Content distribution | Minor, no ADR |
| Code snippet approach | Learning experience | No (follows Module 1 pattern) |

No significant architectural decisions requiring ADR were identified. Module 2 follows patterns established in Module 1.

## Complexity Tracking

> No constitution violations requiring justification.

The module follows established patterns from Module 1 and does not introduce new architectural complexity.

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Execute Phase 2 (Foundation) tasks
3. Execute Phase 3 (Content Development) per chapter
4. Execute Phase 4 (Synthesis) for finalization
5. Validate against success criteria
