---
id: 0003
title: Architectural Planning for RAG Chatbot UI
stage: plan
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: 007-rag-chatbot-ui
branch: 007-rag-chatbot-ui
user: giaic-student
command: /sp.plan
labels: ["architecture", "planning", "rag-chatbot", "ui-design", "docusaurus-integration"]
links:
  spec: specs/007-rag-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-rag-chatbot-ui/plan.md
  - specs/007-rag-chatbot-ui/research.md
  - specs/007-rag-chatbot-ui/data-model.md
  - specs/007-rag-chatbot-ui/quickstart.md
  - specs/007-rag-chatbot-ui/contracts/mock-api.ts
  - CLAUDE.md (agent context updated)
tests:
  - Constitution Check: 6 gates evaluated (5 PASS, 1 PARTIAL with justification)
  - Phase 0 Research: 5 technology decisions documented
  - Phase 1 Data Model: 3 entities defined with state management
  - Phase 1 Contracts: Mock API interface created for future backend integration
---

## Prompt

```text
RAG Chatbot UI for Docusaurus Book

Create:
- Architecture sketch showing:
  - Docusaurus frontend
  - Embedded chatbot UI (React)
  - Integration boundary with RAG backend (API only, no backend logic)
- UI section structure and component breakdown
- Implementation and validation plan for chatbot UI integration
- Quality and acceptance validation strategy

Decisions needing documentation:
- (open/close, send, receive, loading states)
   - Validate context-awareness flow (page or selected text)

4. Synthesis
   - Integrate chatbot UI consistently across the site
   - Normalize styles and remove redundant UI logic

Testing & validation strategy:
- UI validation:
  - Chatbot renders correctly on all pages
  - Responsive behavior on desktop and mobile
- Interaction validation:
  - Messages send and render correctly
  - Context is visibly included in prompts
- Theme validation:
  - Light and dark mode styling matches site theme
- Spec compliance:
  - No backend logic implemented
  - No scope creep beyond UI

Acceptance criteria:
- Chatbot UI is embedded and accessible in Docusaurus
- UI uses shadcn-chatbot-kit components successfully
- Context-aware interaction flow is clearly supported
- Styling is consistent with the Physical AI book theme
- Ready to proceed to /sp.task
```

## Response snapshot

Executed complete `/sp.plan` workflow following Spec-Kit Plus architecture planning phase:

**1. Technical Context & Constitution Check (plan.md)**:
   - Resolved "NEEDS CLARIFICATION" items: markdown library, testing strategy, Docusaurus integration pattern, storage fallback, error boundary
   - Evaluated 6 constitution gates: 5 PASS, 1 PARTIAL (RAG backend integration deferred by design)
   - Justified complexity tracking: Select-text-to-ask deferred to future enhancement, UI-only scope separates concerns from backend

**2. Phase 0: Research & Technology Selection (research.md)**:
   - **Markdown Library**: Selected react-markdown + rehype-prism-plus (35KB, safe by default, Prism.js compatible)
   - **Testing Strategy**: Manual testing for MVP with comprehensive validation checklist; automated testing post-launch
   - **Docusaurus Integration**: Swizzle Root.tsx in wrapper mode (official pattern, stable across upgrades)
   - **Storage Fallback**: Try-catch on setItem() with in-memory Map fallback for private browsing/quota exceeded
   - **Error Boundary**: Class component with reload button (React 18 best practice, user recovery)

**3. Phase 1: Data Model & Contracts (data-model.md, contracts/mock-api.ts)**:
   - **Entities**: Message (id, content, role, timestamp, loadingState), Conversation State (messages[], inputValue, isPanelOpen, isLoading, storageType), Storage Abstraction (session/memory adapters)
   - **State Management**: useChatState hook with persistence to session storage (or in-memory fallback)
   - **API Contract**: TypeScript interface (ChatAPI, SendMessageRequest, SendMessageResponse) with MockChatAPI implementation providing canned responses for UI development

**4. Component Architecture**:
   - 10 React components defined: ChatBot (container), ChatToggleButton, ChatPanel, ChatHeader, MessageList, Message, MessageInput, WelcomeMessage, LoadingIndicator, ErrorBoundary
   - 4 custom hooks: useChatState, useSessionStorage, useTheme, useResponsive
   - 2 services: mockChatService, storageService
   - Clear separation of concerns: UI components → hooks → services → storage adapters

**5. Project Structure**:
   - `frontend/` directory isolates chatbot from Docusaurus internals
   - Integration via swizzled `src/theme/Root.tsx` (global component injection)
   - Modular, testable architecture with clear import paths

**6. Validation Strategy (quickstart.md)**:
   - 12-item acceptance checklist covering UI, responsive, theme, accessibility, performance, and storage persistence
   - Step-by-step integration guide with troubleshooting section
   - Mock-to-production replacement guide for future RAG backend integration

**7. Agent Context Update**:
   - Ran `.specify/scripts/bash/update-agent-context.sh claude`
   - Added JavaScript (ES2020+), React 18.x, Session storage to CLAUDE.md active technologies

## Outcome

- ✅ Impact: Complete architectural plan with Phase 0 (research) and Phase 1 (data model, contracts, quickstart) delivered; ready for `/sp.tasks` generation
- 🧪 Tests: Constitution Check validated (6 gates), research decisions justified (5 technology choices), data model validated (3 entities with state transitions)
- 📁 Files: Created plan.md (599 lines), research.md, data-model.md, quickstart.md, contracts/mock-api.ts; updated CLAUDE.md
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks in red-green-refactor format
- 🧠 Reflection: User's input emphasized UI-only scope, context-awareness prep, and theme consistency - all addressed in plan with mock API boundary for future backend integration; architecture sketch provides visual clarity on integration points

## Evaluation notes (flywheel)

- Failure modes observed: None - workflow executed cleanly through Phase 0 and Phase 1; all NEEDS CLARIFICATION items resolved
- Graders run and results (PASS/FAIL):
  - Constitution Check: 5/6 PASS, 1/6 PARTIAL (justified - RAG backend intentionally out of scope)
  - Research Decisions: 5/5 PASS (all technology choices justified with rationale and rejected alternatives)
  - Data Model Completeness: 3/3 entities defined PASS, state transitions documented PASS
  - Contract Validity: TypeScript interface PASS, mock implementation PASS, production replacement guide PASS
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided architecture requirements (integration boundary, component breakdown, validation strategy)
- Next experiment (smallest change to try): Consider adding ADR suggestion for key architecture decisions (e.g., Swizzle vs Plugin, react-markdown vs Docusaurus MDX) to improve decision traceability
