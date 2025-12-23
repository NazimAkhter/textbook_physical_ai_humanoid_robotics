---
id: 001
title: Initial Constitution Ratification
stage: constitution
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "governance", "project-setup", "physical-ai", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (governance document, no executable tests)
---

## Prompt

/sp.constitution

Project: Unified Educational Book & RAG Chatbot
Theme: Physical AI, Humanoid Robotics, and Vision-Language-Action Systems

Core Principles:
- Spec-Driven Development:
  All outputs must strictly follow Spec-Kit Plus workflows (/sp.constitution → /sp.plan → /sp.task → /sp.implement).
  Any output that violates this sequence is invalid.

- Educational Efficacy:
  Content must explicitly connect theoretical AI concepts to hands-on robotics workflows
  using ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action (VLA) models.

- Seamless Documentation–Agent Integration:
  Static content (Docusaurus) and dynamic AI interaction (RAG chatbot) must be tightly coupled,
  with shared source content as the single ground truth.

- Reproducibility by Default:
  All code examples (Python/rclpy, URDF, Gazebo, Isaac) must be syntactically valid,
  executable, and suitable for local or cloud deployment.

Technical Standards:
- Frontend / Book:
  - Docusaurus (latest stable)
  - Mobile-responsive UI
  - Deployed on Vercel

- Chatbot Backend:
  - FastAPI
  - Runtime usage of OpenAI Agents SDK and/or Gemini ChatKit is permitted
  - Code generation must be performed exclusively via Claude Code + Spec-Kit Plus

- Data & Retrieval:
  - Neon Serverless Postgres for conversation metadata
  - Qdrant Cloud (Free Tier) for vector storage
  - Embedding strategy must explicitly optimize for free-tier limits

Content Structure (Strict):
1. ROS 2 as the Robotic Nervous System
2. Digital Twins & Simulation
3. NVIDIA Isaac & Physical AI
4. Vision-Language-Action (VLA) Systems
5. Capstone Project

Module order, naming, and scope are fixed and must not be altered.

Functional Requirements:
- Embedded RAG chatbot within the Docusaurus UI
- "Select-text-to-ask" interaction:
  User-selected text in the book must be injected as context into chatbot queries
- Retrieval must be limited to book content stored in Qdrant

Constraints:
- Generation Tooling:
  Only Claude Code and Spec-Kit Plus may be used for content or code generation
- No features beyond documented scope (e.g., auth, voice, multilingual support) unless explicitly specified
- Backend hosting must be compatible with FastAPI (e.g., Railway or Render)

Success Criteria (Pass/Fail)

## Response snapshot

Created initial project constitution v1.0.0 for "Unified Educational Book & RAG Chatbot" with theme "Physical AI, Humanoid Robotics, and Vision-Language-Action Systems".

Constitution includes:
- 6 Core Principles: Spec-Driven Development, Educational Efficacy, Seamless Documentation-Agent Integration, Reproducibility by Default, Technical Standards, Constraints & Compliance
- Fixed Content Structure: 5 modules (ROS 2, Digital Twins, NVIDIA Isaac, VLA Systems, Capstone)
- Technical Stack: Docusaurus/Vercel (frontend), FastAPI (backend), Neon Postgres + Qdrant Cloud (data)
- Functional Requirements: Embedded RAG chatbot, select-text-to-ask feature
- Success Criteria: 6 pass/fail conditions
- Governance: Amendment process, compliance rules, semantic versioning

All templates validated as compatible. No follow-up TODOs required.

## Outcome

- Impact: Established foundational governance for entire project lifecycle
- Tests: N/A (governance document)
- Files: .specify/memory/constitution.md (created/updated)
- Next prompts: /sp.specify for first feature, /sp.plan after spec approval
- Reflection: User provided comprehensive requirements; all placeholders filled without ambiguity

## Evaluation notes (flywheel)

- Failure modes observed: None - user input was complete and unambiguous
- Graders run and results (PASS/FAIL): Manual validation PASS - all placeholders filled, no bracket tokens remaining
- Prompt variant (if applicable): Standard /sp.constitution workflow
- Next experiment (smallest change to try): N/A - initial constitution successfully ratified
