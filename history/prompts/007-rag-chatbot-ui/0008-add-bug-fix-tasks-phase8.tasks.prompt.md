---
id: 0008
title: Add Bug Fix Tasks Phase 8
stage: tasks
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: 007-rag-chatbot-ui
branch: 007-rag-chatbot-ui
user: user
command: sp.tasks
labels: ["bug-fix", "tasks", "props-mismatch", "phase8"]
links:
  spec: specs/007-rag-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-rag-chatbot-ui/tasks.md
tests:
  - N/A (task generation only)
---

## Prompt

sp.tasks Create Bug Fix Task

## Response snapshot

**Phase 8: Bug Fixes Added to tasks.md**

Created a new phase in tasks.md to address the critical props mismatch bug identified in `/sp.analyze`. The bug prevents the chatbot panel from displaying when the toggle button is clicked.

**Tasks Added (T086-T095)**:

**Bug Fix Tasks (7 parallel tasks)**:
- T086: Fix missing `isOpen` prop (CRITICAL - blocks panel rendering)
- T087: Fix `onClear` → `onClearConversation` prop name mismatch
- T088: Fix `isStorageFallback` type mismatch (boolean → StorageType)
- T089: Remove or add interface support for `inputRef` prop
- T090: Remove or add interface support for `welcomeTitle` prop
- T091: Remove or add interface support for `welcomeMessage` prop
- T092: Remove redundant `isOpen` conditional check

**Validation Tasks (3 sequential tasks)**:
- T093: Build verification after props fixes
- T094: Functional test - verify toggle button opens panel
- T095: Regression test - verify all existing functionality intact

**Task Organization**:
- All bug fix tasks (T086-T092) are parallelizable [P] - can be fixed simultaneously
- Validation tasks are sequential (build → functional → regression)
- Total: 10 new tasks (7 parallel, 3 sequential)

**Updated Metrics**:
- Total tasks: 85 → 95 (+10 bug fix tasks)
- Parallel opportunities: 43 → 50 (+7)
- New Phase: Phase 8 (Bug Fixes)
- Status: Changed from "Complete" to "Bug Fixes Pending"

**Task Format Compliance**: All tasks follow required checklist format:
- ✅ Checkbox prefix `- [ ]`
- ✅ Sequential Task IDs (T086-T095)
- ✅ Parallel markers [P] where applicable
- ✅ File paths and line numbers specified
- ✅ Clear acceptance criteria in validation tasks

## Outcome

- ✅ Impact: Added Phase 8 with 10 bug fix tasks addressing critical props mismatch. Tasks are immediately actionable with exact file paths and line numbers.
- 🧪 Tests: 3 validation tasks (build, functional, regression) ensure bug fix quality
- 📁 Files: Modified tasks.md (added Phase 8, updated task summary, parallel execution examples)
- 🔁 Next prompts: `/sp.implement phase 8` to execute bug fixes
- 🧠 Reflection: Bug fix tasks demonstrate importance of integration testing. Props mismatch not caught by TypeScript strict mode or build process, only by runtime functional testing.

## Evaluation notes (flywheel)

- Failure modes observed: None - task generation successful
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks follow checklist format)
- Prompt variant (if applicable): Standard task generation workflow adapted for bug fix phase
- Next experiment (smallest change to try): Execute T086 (add missing isOpen prop) and verify panel renders
