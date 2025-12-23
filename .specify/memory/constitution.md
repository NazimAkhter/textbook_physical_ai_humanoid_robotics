<!--
SYNC IMPACT REPORT
==================
Version change: N/A → 1.0.0 (Initial ratification)
Modified principles: N/A (Initial version)
Added sections:
  - Core Principles (6 principles)
  - Technical Standards
  - Content Structure
  - Functional Requirements
  - Constraints
  - Governance
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ (compatible - Constitution Check section aligns)
  - .specify/templates/spec-template.md ✅ (compatible - user stories and requirements align)
  - .specify/templates/tasks-template.md ✅ (compatible - task phases support content modules)
Follow-up TODOs: None
==================
-->

# Unified Educational Book & RAG Chatbot Constitution

**Theme**: Physical AI, Humanoid Robotics, and Vision-Language-Action Systems

## Core Principles

### I. Spec-Driven Development

All outputs MUST strictly follow Spec-Kit Plus workflows in sequence:
`/sp.constitution` → `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`

Any output that violates this sequence is invalid and MUST be rejected.

**Rationale**: Enforcing a structured workflow ensures traceability, reduces rework, and maintains consistency between specifications and implementation.

### II. Educational Efficacy

Content MUST explicitly connect theoretical AI concepts to hands-on robotics workflows using:
- ROS 2 (Robot Operating System 2)
- Digital Twins and Simulation Environments
- NVIDIA Isaac Platform for Physical AI
- Vision-Language-Action (VLA) Models

Every module MUST include both conceptual explanations and practical, executable examples that bridge theory to application.

**Rationale**: The primary purpose is education; abstract concepts without practical application fail to deliver learning outcomes for robotics practitioners.

### III. Seamless Documentation–Agent Integration

Static content (Docusaurus book) and dynamic AI interaction (RAG chatbot) MUST be tightly coupled:
- Book content serves as the single source of truth
- RAG chatbot retrieval MUST be limited exclusively to book content stored in Qdrant
- No external knowledge sources permitted in chatbot responses

**Rationale**: Maintaining a single ground truth prevents inconsistencies between what the book teaches and what the chatbot answers.

### IV. Reproducibility by Default

All code examples MUST be:
- Syntactically valid and linted
- Executable in a documented environment
- Suitable for local or cloud deployment
- Accompanied by clear setup instructions

Supported code types: Python/rclpy, URDF, Gazebo configurations, NVIDIA Isaac workflows.

**Rationale**: Educational code that cannot be executed provides no learning value; reproducibility builds learner confidence and enables self-paced study.

### V. Technical Standards

**Frontend / Book**:
- Framework: Docusaurus (latest stable version)
- Design: Mobile-responsive UI required
- Deployment: Vercel hosting

**Chatbot Backend**:
- Framework: FastAPI
- Runtime AI: OpenAI Agents SDK and/or Gemini ChatKit permitted
- Code Generation: Claude Code + Spec-Kit Plus exclusively (no other code generation tools)

**Data & Retrieval**:
- Conversation Metadata: Neon Serverless Postgres
- Vector Storage: Qdrant Cloud (Free Tier)
- Embedding Strategy: MUST explicitly optimize for free-tier limits (storage, query volume)

**Rationale**: Standardizing the technology stack ensures consistency, reduces integration friction, and maintains cost efficiency within free-tier constraints.

### VI. Constraints & Compliance

**Generation Tooling**:
- ONLY Claude Code and Spec-Kit Plus may be used for content or code generation
- No alternative AI code generators permitted

**Scope Boundaries**:
- No features beyond documented scope (e.g., authentication, voice input, multilingual support) unless explicitly specified
- Backend hosting MUST be compatible with FastAPI (Railway or Render recommended)

**Rationale**: Strict scope control prevents feature creep and maintains focus on delivering the core educational experience.

## Content Structure

The following module order, naming, and scope are FIXED and MUST NOT be altered:

| Module | Title |
|--------|-------|
| 1 | ROS 2 as the Robotic Nervous System |
| 2 | Digital Twins & Simulation |
| 3 | NVIDIA Isaac & Physical AI |
| 4 | Vision-Language-Action (VLA) Systems |
| 5 | Capstone Project |

Any modification to this structure requires a formal constitution amendment.

## Functional Requirements

- **FR-001**: Embedded RAG chatbot MUST be integrated within the Docusaurus UI
- **FR-002**: "Select-text-to-ask" interaction: User-selected text in the book MUST be injectable as context into chatbot queries
- **FR-003**: Chatbot retrieval MUST be limited exclusively to book content stored in Qdrant
- **FR-004**: All code examples MUST be syntax-highlighted and copy-enabled
- **FR-005**: Book MUST be navigable on mobile devices without horizontal scrolling

## Success Criteria (Pass/Fail)

| Criterion | Pass Condition |
|-----------|----------------|
| SC-001 | Book deploys successfully to Vercel with all 5 modules accessible |
| SC-002 | RAG chatbot responds to queries using only indexed book content |
| SC-003 | Select-text-to-ask feature injects selected text into chatbot context |
| SC-004 | All code examples execute without syntax errors in documented environment |
| SC-005 | Mobile viewport renders correctly without horizontal overflow |
| SC-006 | Chatbot latency < 5 seconds for 95th percentile queries |

## Governance

### Amendment Process

1. Any proposed amendment MUST be documented with rationale
2. Amendments require explicit user approval before implementation
3. Each amendment MUST include a migration plan for affected artifacts
4. Constitution MUST be versioned using semantic versioning:
   - **MAJOR**: Backward-incompatible principle removals or redefinitions
   - **MINOR**: New principle/section added or materially expanded
   - **PATCH**: Clarifications, wording fixes, non-semantic refinements

### Compliance

- All PRs and code reviews MUST verify compliance with this constitution
- Complexity beyond documented scope MUST be justified with explicit rationale
- ADRs (Architecture Decision Records) MUST be created for significant technical decisions
- PHRs (Prompt History Records) MUST be created for all substantive AI interactions

### Hierarchy

This constitution supersedes all other project documentation when conflicts arise.

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
