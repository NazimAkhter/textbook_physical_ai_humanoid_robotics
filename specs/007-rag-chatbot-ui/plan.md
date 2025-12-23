# Implementation Plan: RAG Chatbot UI for Physical AI & Humanoid Robotics Book

**Branch**: `007-rag-chatbot-ui` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-rag-chatbot-ui/spec.md`

## Summary

Build an embedded RAG chatbot UI layer for the Physical AI & Humanoid Robotics Docusaurus book. The chatbot provides context-aware Q&A for readers using a React-based interface with responsive design (desktop side panel, mobile bottom sheet). Scope is strictly UI-only—no RAG backend, vector database, or API implementation. Mock responses will be used during development. The interface must integrate seamlessly with Docusaurus 3.x theming, support markdown rendering with syntax highlighting, persist conversation history via session storage (with in-memory fallback), and meet WCAG 2.1 Level AA accessibility standards.

## Technical Context

**Language/Version**: JavaScript (ES2020+), React 18.x
**Primary Dependencies**:
- Docusaurus 3.x (existing)
- React 18.x (provided by Docusaurus)
- react-markdown or similar (markdown rendering) - NEEDS CLARIFICATION: specific library choice
- Prism.js or highlight.js (syntax highlighting) - Docusaurus built-in Prism preferred
- CSS Modules or Infima CSS variables (scoped styling)

**Storage**: Session storage (browser API) for conversation persistence; in-memory fallback for private browsing/quota exceeded scenarios
**Testing**: Manual UI testing against acceptance criteria; responsive testing on desktop (≥996px), tablet (768-996px), mobile (<768px) - NEEDS CLARIFICATION: automated testing strategy (Jest, React Testing Library, Playwright)
**Target Platform**: Modern evergreen browsers (Chrome, Firefox, Safari, Edge) with ES2020+ support
**Project Type**: Web application (frontend-only, embedded in existing Docusaurus site)
**Performance Goals**:
- <50KB bundle size for chatbot components (lazy-loaded)
- <100ms UI interaction latency (typing, scrolling, closing)
- <200ms theme change adaptation
- Support 20+ messages without scroll lag

**Constraints**:
- No RAG backend implementation (out of scope)
- No API integration (mock responses only)
- Must use Docusaurus built-in libraries where possible
- Strict adherence to Infima CSS variables for theming
- <50KB bundle impact (lazy-loaded)
- WCAG 2.1 Level AA compliance required

**Scale/Scope**:
- Single chatbot UI component integrated across all documentation pages
- 34 functional requirements (FR-001 to FR-034)
- 3 responsive breakpoints (mobile, tablet, desktop)
- 2 theme modes (light, dark)
- 4 prioritized user stories (P1-P4)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Gate 1: Spec-Driven Development (Principle I)

✅ **PASS**: This plan follows `/sp.specify` → `/sp.clarify` → `/sp.plan` sequence.

### Gate 2: Educational Efficacy (Principle II)

✅ **PASS**: Chatbot UI enhances educational experience by providing contextual Q&A for Physical AI & Humanoid Robotics content. Supports learner comprehension without leaving documentation.

### Gate 3: Seamless Documentation–Agent Integration (Principle III)

⚠️ **PARTIAL**: UI layer prepares for RAG integration, but actual RAG backend (retrieval from Qdrant, book content indexing) is out of scope for this feature. Mock responses used for UI development.

**Justification**: This feature focuses exclusively on UI layer as specified. RAG backend integration will be addressed in a separate feature. This aligns with spec FR requirements (UI-only scope).

### Gate 4: Reproducibility by Default (Principle IV)

✅ **PASS**: React components will be documented with clear integration instructions. Example usage patterns will be provided in quickstart.md. No executable code examples required (UI components, not algorithms).

### Gate 5: Technical Standards (Principle V)

✅ **PASS**:
- Frontend: Docusaurus 3.x (existing), React 18.x ✅
- Design: Mobile-responsive (3 breakpoints defined) ✅
- Deployment: Vercel hosting (existing) ✅
- Code Generation: Claude Code + Spec-Kit Plus exclusively ✅

### Gate 6: Constraints & Compliance (Principle VI)

✅ **PASS**:
- Generation Tooling: Claude Code + Spec-Kit Plus only ✅
- Scope Boundaries: No authentication, voice input, multilingual support, or backend hosting (all out of scope per spec) ✅
- Backend Compatibility: N/A (UI-only feature) ✅

### Constitution Functional Requirements Alignment

| Constitution FR | Spec Alignment | Status |
|-----------------|----------------|--------|
| FR-001: Embedded RAG chatbot in Docusaurus UI | FR-001 (toggle button), FR-002 (chat panel) | ✅ Covered |
| FR-002: Select-text-to-ask interaction | NOT in current spec | ⚠️ Deferred (future enhancement) |
| FR-003: Chatbot retrieval limited to book content | Out of scope (backend concern) | ⚠️ N/A for UI layer |
| FR-004: Syntax-highlighted, copy-enabled code | FR-007 (syntax highlighting in responses) | ✅ Covered |
| FR-005: Mobile navigation without horizontal scroll | FR-012 (responsive design) | ✅ Covered |

**Note**: FR-002 (select-text-to-ask) is noted for future enhancement but not required for MVP. Current spec focuses on standard chat interface.

### Constitution Success Criteria Alignment

| Constitution SC | Spec Alignment | Status |
|-----------------|----------------|--------|
| SC-001: Book deploys to Vercel | Deployment scope (outside UI feature) | ⚠️ N/A |
| SC-002: RAG chatbot responds using book content | Backend scope (outside UI feature) | ⚠️ N/A |
| SC-003: Select-text-to-ask feature | Future enhancement | ⚠️ Deferred |
| SC-004: Code examples execute without errors | FR-007 (display only) | ✅ Partial (UI renders code, execution out of scope) |
| SC-005: Mobile viewport without overflow | FR-012, FR-024 (responsive design) | ✅ Covered |
| SC-006: Chatbot latency <5s (95th percentile) | SC-001 (5s to first question) | ✅ Covered (UI interaction latency) |

**Re-check Required After Phase 1**: Verify that component design does not introduce scope creep beyond UI layer.

## Project Structure

### Documentation (this feature)

```text
specs/007-rag-chatbot-ui/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (markdown library, testing strategy, Docusaurus integration patterns)
├── data-model.md        # Phase 1 output (Message, Conversation State entities)
├── quickstart.md        # Phase 1 output (integration guide for developers)
├── contracts/           # Phase 1 output (mock API interface for future backend integration)
│   └── mock-api.ts      # TypeScript interface for chat API (send message, receive response)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Option: Web application (Docusaurus site with embedded React components)

frontend/                       # New directory for chatbot UI components
├── src/
│   ├── components/
│   │   ├── ChatBot/
│   │   │   ├── ChatBot.tsx              # Main container component
│   │   │   ├── ChatToggleButton.tsx     # Floating toggle button
│   │   │   ├── ChatPanel.tsx            # Panel/modal container (responsive)
│   │   │   ├── ChatHeader.tsx           # Header with close button
│   │   │   ├── MessageList.tsx          # Scrollable message history
│   │   │   ├── Message.tsx              # Individual message bubble
│   │   │   ├── MessageInput.tsx         # Text input with submit
│   │   │   ├── WelcomeMessage.tsx       # Empty state welcome
│   │   │   ├── LoadingIndicator.tsx     # Response loading spinner
│   │   │   └── ErrorBoundary.tsx        # Error fallback UI
│   │   └── ChatBot/styles/
│   │       ├── ChatBot.module.css       # Scoped component styles
│   │       ├── ChatPanel.module.css     # Responsive panel styles
│   │       ├── Message.module.css       # Message bubble styles
│   │       └── theme-variables.css      # Infima CSS variable mappings
│   ├── hooks/
│   │   ├── useChatState.ts              # Conversation state management
│   │   ├── useSessionStorage.ts         # Session storage with fallback
│   │   ├── useTheme.ts                  # Docusaurus theme detection
│   │   └── useResponsive.ts             # Breakpoint detection
│   ├── services/
│   │   ├── mockChatService.ts           # Mock API for development
│   │   └── storageService.ts            # Storage abstraction (session/memory)
│   ├── types/
│   │   ├── message.ts                   # Message type definitions
│   │   └── chatState.ts                 # Conversation state types
│   └── utils/
│       ├── markdown.ts                  # Markdown rendering utilities
│       └── errorLogger.ts               # Error logging (console + optional hook)
└── tests/                       # Test suite (if automated testing adopted)
    └── components/
        └── ChatBot/
            ├── ChatBot.test.tsx
            ├── MessageList.test.tsx
            └── MessageInput.test.tsx

# Existing Docusaurus structure (integration points)
docs/                            # Existing documentation content
src/
├── theme/                       # Docusaurus theme customization (swizzling)
│   └── Root.tsx                 # Global wrapper for ChatBot component injection
└── pages/                       # Custom pages (if needed)

docusaurus.config.js             # Docusaurus configuration (lazy loading config)
package.json                     # Dependencies (react-markdown, etc.)
```

**Structure Decision**: Web application (frontend-only) structure selected because this is an embedded UI component for an existing Docusaurus site. The `frontend/` directory isolates chatbot components from Docusaurus internals, making the feature modular and testable. Integration occurs via Docusaurus theme customization (swizzling `Root.tsx` to inject `<ChatBot />` globally).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| FR-002 (select-text-to-ask) deferred | Spec focuses on standard chat interface; text selection adds significant UX/interaction complexity | Including text selection would delay MVP delivery; can be added as enhancement post-launch without blocking core functionality |
| Constitution SC-002, SC-003 partially unmet | This feature is UI-only; RAG backend integration is separate feature | Attempting to build RAG backend alongside UI violates single-responsibility and exceeds specified scope; mock responses enable UI development independently |

**Rationale**: The spec explicitly scopes this feature as "UI layer only (React components)" with "Not building: RAG backend, vector database, or LLM logic, API integration or network request handling." Constitution principles are upheld; backend integration will follow in subsequent feature.

---

## Phase 0: Research & Technology Selection

**Prerequisites**: Constitution Check passed

**Goal**: Resolve all "NEEDS CLARIFICATION" items from Technical Context by researching best practices and documenting decisions.

### Research Tasks

1. **Markdown Rendering Library Selection**
   - **Question**: Which markdown library best supports Docusaurus integration and syntax highlighting?
   - **Options**: react-markdown, marked + DOMPurify, Docusaurus built-in MDX components
   - **Research Focus**: Bundle size, Prism.js compatibility, XSS safety, React 18 support

2. **Automated Testing Strategy**
   - **Question**: Should we adopt automated testing (Jest, React Testing Library, Playwright) or rely on manual acceptance testing?
   - **Research Focus**: ROI for UI component testing, CI/CD integration effort, coverage for responsive/accessibility requirements

3. **Docusaurus Integration Pattern**
   - **Question**: Best practice for injecting global React component (ChatBot) into Docusaurus?
   - **Options**: Swizzle Root.tsx, plugin architecture, direct DOM injection
   - **Research Focus**: Official Docusaurus recommendations, maintainability across Docusaurus upgrades

4. **Session Storage Fallback Implementation**
   - **Question**: How to reliably detect session storage unavailability and switch to in-memory storage?
   - **Research Focus**: Private browsing detection patterns, quota exceeded handling, graceful degradation best practices

5. **Error Boundary Best Practices**
   - **Question**: How to implement React error boundary for chat component with reload functionality?
   - **Research Focus**: React 18 error boundary patterns, user-friendly fallback UI, error logging integration

**Output**: `research.md` documenting decisions, rationale, and alternatives for each research task.

---

## Phase 1: Data Model & Contracts

**Prerequisites**: Phase 0 research complete, all NEEDS CLARIFICATION resolved

### Task 1.1: Data Model Design

**Output**: `data-model.md` with entities and state management

**Entities** (from spec Key Entities section):

1. **Message**
   - Fields: id (string), content (string), role (user | assistant), timestamp (Date), loadingState (boolean)
   - Relationships: Part of messages[] array in Conversation State
   - Validation: content non-empty, role enum, timestamp valid Date

2. **Conversation State**
   - Fields: messages (Message[]), inputValue (string), isPanelOpen (boolean), isLoading (boolean), storageType (session | memory)
   - Lifecycle: Created on first toggle, persisted in session storage (or memory fallback), cleared on user action or browser session end
   - State Transitions: closed → open, sending → waiting for response → displaying response

3. **Storage Abstraction**
   - Interface: getConversation(), saveConversation(state), clearConversation()
   - Implementations: SessionStorageAdapter, InMemoryStorageAdapter
   - Fallback Logic: Detect storage availability on init, switch to memory if unavailable

### Task 1.2: API Contracts (Mock)

**Output**: `contracts/mock-api.ts` - TypeScript interface for future RAG backend

```typescript
// contracts/mock-api.ts
interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string; // ISO 8601
}

interface SendMessageRequest {
  message: string;
  conversationHistory: ChatMessage[];
  context?: {
    currentPage?: string;
    selectedText?: string; // Future: select-text-to-ask
  };
}

interface SendMessageResponse {
  reply: string;
  messageId: string;
  timestamp: string;
}

interface ChatAPI {
  sendMessage(request: SendMessageRequest): Promise<SendMessageResponse>;
}

// Mock implementation for UI development
class MockChatAPI implements ChatAPI {
  async sendMessage(request: SendMessageRequest): Promise<SendMessageResponse> {
    // Simulate 500-1500ms delay
    await new Promise(resolve => setTimeout(resolve, Math.random() * 1000 + 500));

    // Return canned responses based on keywords
    const message = request.message.toLowerCase();
    let reply = "I'm a mock chatbot. The real RAG backend will provide contextual answers from the Physical AI & Humanoid Robotics book.";

    if (message.includes('vla')) {
      reply = "**Vision-Language-Action (VLA)** systems combine computer vision, natural language understanding, and robotic action planning. They enable robots to perceive their environment, interpret human instructions, and execute physical tasks.";
    } else if (message.includes('slam')) {
      reply = "**SLAM (Simultaneous Localization and Mapping)** is a technique used in robotics to build a map of an unknown environment while simultaneously tracking the robot's location within that map.";
    }

    return {
      reply,
      messageId: `msg-${Date.now()}`,
      timestamp: new Date().toISOString()
    };
  }
}
```

### Task 1.3: Component Architecture

**Output**: Component breakdown with props/state

1. **ChatBot** (Container)
   - State: useChatState (messages, isOpen, storage type)
   - Responsibilities: Global state management, theme integration
   - Children: ChatToggleButton, ChatPanel (conditional render)

2. **ChatToggleButton**
   - Props: onClick, isOpen
   - Responsibilities: Fixed-position button, accessibility (ARIA label, keyboard nav)

3. **ChatPanel**
   - Props: isOpen, onClose, messages, onSendMessage
   - Responsibilities: Responsive layout (desktop 400px panel, mobile 60-70% bottom sheet), focus trap, ESC key handler

4. **MessageList**
   - Props: messages[], isLoading
   - Responsibilities: Auto-scroll, scroll position stability, 20+ message performance

5. **Message**
   - Props: content, role, timestamp
   - Responsibilities: Markdown rendering, syntax highlighting, distinct styling (user vs assistant)

6. **MessageInput**
   - Props: value, onChange, onSubmit, disabled
   - Responsibilities: Multiline support (shift+enter), empty validation, submit on Enter

7. **WelcomeMessage**
   - Props: message text (from FR-033)
   - Responsibilities: Empty state display, centered/lighter styling

### Task 1.4: Update Agent Context

**Action**: Run `.specify/scripts/bash/update-agent-context.sh claude`

**Purpose**: Add new technologies from this plan to agent-specific context file:
- react-markdown (or selected markdown library from research)
- Session Storage API patterns
- Docusaurus theme integration (swizzling)

---

## Phase 2: Implementation Planning (Tasks Generation)

**Note**: This phase is executed by `/sp.tasks` command, NOT `/sp.plan`. Documented here for completeness.

**Prerequisites**: Phase 1 complete (data-model.md, contracts/, quickstart.md created)

**Output**: `tasks.md` with implementation tasks in red-green-refactor format

**Task Categories** (to be generated by `/sp.tasks`):

1. **Foundation Tasks** (Red Phase)
   - Setup frontend/ directory structure
   - Configure Docusaurus integration (swizzle Root.tsx)
   - Install dependencies (markdown library from research)
   - Create TypeScript types (Message, Conversation State)

2. **Core UI Components** (Red → Green)
   - ChatToggleButton (FR-001, FR-016, FR-018)
   - ChatPanel responsive layout (FR-024, FR-031, FR-032)
   - MessageList with auto-scroll (FR-008, FR-009)
   - Message with markdown rendering (FR-006, FR-007)
   - MessageInput with validation (FR-003, FR-015)

3. **State Management** (Red → Green)
   - useChatState hook (messages, isOpen, loading)
   - useSessionStorage with fallback (FR-025, FR-026, FR-027)
   - mockChatService integration

4. **Theming & Responsiveness** (Red → Green)
   - Infima CSS variable integration (FR-011)
   - Responsive breakpoints (FR-012, FR-024, FR-031, FR-032)
   - Theme transition smoothness (SC-006: <200ms)

5. **Accessibility** (Red → Green)
   - Keyboard navigation (FR-016, FR-017, FR-018)
   - Focus management (FR-017)
   - ARIA labels (SC-007: WCAG 2.1 AA)

6. **Error Handling & Observability** (Red → Green)
   - Error boundary (FR-028, edge case: rendering failure)
   - Error logging (FR-028, FR-029, FR-030)
   - Storage failure notification (FR-027)

7. **Performance Optimization** (Refactor)
   - Lazy loading (FR-019, SC-003: <50KB)
   - Bundle analysis and tree-shaking
   - Scroll performance validation (SC-005: 20+ messages)

8. **Validation** (Green → Acceptance)
   - Manual testing against acceptance scenarios (User Stories P1-P4)
   - Responsive testing (320px to 3840px per SC-004)
   - Theme switching validation (SC-006: <200ms)
   - Accessibility audit (SC-007: WCAG 2.1 AA)

---

## Quickstart Integration

**Output**: `quickstart.md` - Developer integration guide

**Contents** (to be generated in Phase 1):
1. Prerequisites (Docusaurus 3.x, React 18.x, Node.js version)
2. Installation steps (copy frontend/ directory, npm install dependencies)
3. Docusaurus integration (swizzle Root.tsx, import ChatBot)
4. Configuration options (error callback hook, theme customization)
5. Mock API replacement guide (for future RAG backend integration)
6. Troubleshooting (common issues, storage fallback testing)

---

## Validation & Acceptance Criteria

### Spec Compliance Checklist

✅ All 34 functional requirements (FR-001 to FR-034) implemented
✅ All 8 success criteria (SC-001 to SC-008) verified
✅ All 3 UX targets (UX-001 to UX-003) validated
✅ 4 user stories (P1-P4) acceptance scenarios passed
✅ No backend logic implemented (UI-only scope maintained)
✅ No scope creep beyond spec boundaries

### Testing Strategy

**Manual UI Testing**:
- [ ] Chatbot toggle button visible and accessible on all documentation pages
- [ ] Chat panel opens smoothly without layout disruption
- [ ] Message submission works with loading indicator
- [ ] Markdown rendering displays correctly with syntax highlighting
- [ ] Clear conversation resets to welcome message
- [ ] Session storage persists across page navigation
- [ ] In-memory fallback works in private browsing mode
- [ ] Error boundary displays fallback UI on component error

**Responsive Testing**:
- [ ] Desktop (≥996px): 400px right-side panel, 80% max-height
- [ ] Tablet (768-996px): 350px right-side panel, 85% max-height
- [ ] Mobile (<768px): Bottom sheet, 60-70% screen height
- [ ] No horizontal scrolling on any viewport (320px minimum)

**Theme Testing**:
- [ ] Light mode: Colors match Infima light theme variables
- [ ] Dark mode: Colors match Infima dark theme variables
- [ ] Theme toggle: Transition completes within 200ms

**Accessibility Testing**:
- [ ] Keyboard navigation: Tab through all interactive elements
- [ ] Focus indicators: WCAG 2.1 Level AA compliant
- [ ] Screen reader: ARIA labels present and descriptive
- [ ] ESC key closes chat panel

**Performance Testing**:
- [ ] Bundle size: ChatBot components <50KB (lazy-loaded)
- [ ] Interaction latency: Typing/scrolling <100ms
- [ ] Theme change: <200ms transition
- [ ] Message rendering: 20+ messages without scroll lag

### Acceptance Criteria (from user input)

✅ Chatbot UI is embedded and accessible in Docusaurus
✅ UI renders correctly on all pages (desktop, tablet, mobile)
✅ Messages send and render correctly (mock responses)
✅ Styling is consistent with Physical AI book theme (Infima CSS variables)
✅ Context-aware interaction flow supported (mock API includes context field for future enhancement)
✅ No backend logic implemented (mock service only)
✅ Ready to proceed to `/sp.tasks`

---

## Architecture Sketch

```text
┌─────────────────────────────────────────────────────────────┐
│                    Docusaurus Frontend                       │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  Documentation Pages (MDX)                            │  │
│  │  - Module 1: ROS 2 Nervous System                     │  │
│  │  - Module 2: Digital Twins                            │  │
│  │  - Module 3: NVIDIA Isaac                             │  │
│  │  - Module 4: VLA Systems                              │  │
│  └───────────────────────────────────────────────────────┘  │
│                           │                                  │
│                           ▼                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  Root.tsx (Swizzled Theme Component)                  │  │
│  │  - Global wrapper for all pages                       │  │
│  │  - Injects <ChatBot /> component                      │  │
│  └───────────────────────────────────────────────────────┘  │
│                           │                                  │
│                           ▼                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  ChatBot Container Component                          │  │
│  │  ┌─────────────────────────────────────────────────┐  │  │
│  │  │  useChatState Hook                              │  │  │
│  │  │  - messages: Message[]                          │  │  │
│  │  │  - isPanelOpen: boolean                         │  │  │
│  │  │  - isLoading: boolean                           │  │  │
│  │  │  - storageType: 'session' | 'memory'            │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  │                                                         │  │
│  │  ┌─────────────────────┐   ┌──────────────────────┐   │  │
│  │  │ ChatToggleButton    │   │ ChatPanel (Conditional)│  │  │
│  │  │ - Fixed position    │   │ ┌──────────────────┐ │   │  │
│  │  │ - Floating FAB      │   │ │ ChatHeader       │ │   │  │
│  │  │ - Accessibility     │   │ │ - Close button   │ │   │  │
│  │  └─────────────────────┘   │ └──────────────────┘ │   │  │
│  │                             │ ┌──────────────────┐ │   │  │
│  │                             │ │ MessageList      │ │   │  │
│  │                             │ │ - Auto-scroll    │ │   │  │
│  │                             │ │ - WelcomeMessage │ │   │  │
│  │                             │ │ - Message[]      │ │   │  │
│  │                             │ │ - LoadingIndicator│ │   │  │
│  │                             │ └──────────────────┘ │   │  │
│  │                             │ ┌──────────────────┐ │   │  │
│  │                             │ │ MessageInput     │ │   │  │
│  │                             │ │ - Textarea       │ │   │  │
│  │                             │ │ - Submit button  │ │   │  │
│  │                             │ │ - Validation     │ │   │  │
│  │                             │ └──────────────────┘ │   │  │
│  │                             └──────────────────────┘   │  │
│  └───────────────────────────────────────────────────────┘  │
│                           │                                  │
│                           ▼                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  Services Layer                                        │  │
│  │  ┌─────────────────────┐   ┌──────────────────────┐   │  │
│  │  │ mockChatService.ts  │   │ storageService.ts    │   │  │
│  │  │ - sendMessage()     │   │ - SessionStorage     │   │  │
│  │  │ - Mock responses    │   │ - InMemory fallback  │   │  │
│  │  │ - 500-1500ms delay  │   │ - Auto-detect        │   │  │
│  │  └─────────────────────┘   └──────────────────────┘   │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                           │
                           │ (Future Integration Boundary)
                           │
                           ▼
         ┌────────────────────────────────────┐
         │   RAG Backend API (Out of Scope)   │
         │   - FastAPI server                 │
         │   - Qdrant vector DB               │
         │   - OpenAI/Gemini LLM              │
         │   - Book content indexing          │
         └────────────────────────────────────┘
```

**Integration Boundary Notes**:
- Mock service returns canned responses during UI development
- Future RAG backend will replace mockChatService with real API client
- ChatAPI interface (contracts/mock-api.ts) defines contract for backend integration
- Context field in SendMessageRequest prepared for page-aware retrieval (future)

---

## Risk Mitigation

### Risk 1: Markdown Library XSS Vulnerability

**Mitigation**: Research phase will evaluate XSS safety (DOMPurify integration, react-markdown built-in sanitization). FR-030 requires privacy-safe error messages; same principle applies to user content sanitization.

### Risk 2: Session Storage Quota Exceeded

**Mitigation**: FR-025, FR-026 already specify graceful degradation to in-memory storage. Research phase will identify reliable quota detection patterns.

### Risk 3: Docusaurus Upgrade Breaking Swizzled Component

**Mitigation**: Quickstart.md will document Docusaurus version dependency. Swizzling Root.tsx is a stable integration pattern per Docusaurus docs. Version pinning in package.json recommended.

### Risk 4: Bundle Size Exceeding 50KB

**Mitigation**: Lazy loading specified in FR-019, SC-003. Phase 1 will establish bundle analysis workflow (webpack-bundle-analyzer). Refactor phase includes tree-shaking optimization.

---

## Next Steps

1. **Immediate**: Begin Phase 0 research (markdown library, testing strategy, Docusaurus integration)
2. **Phase 0 Output**: `research.md` with all NEEDS CLARIFICATION resolved
3. **Phase 1**: Generate `data-model.md`, `contracts/mock-api.ts`, `quickstart.md`
4. **Phase 1**: Run agent context update script
5. **Ready for `/sp.tasks`**: With complete data model and contracts, task generation can proceed

---

**Plan Status**: ✅ Complete and ready for Phase 0 research
**Blocker**: None - all prerequisites met
**Next Command**: Begin Phase 0 research tasks (documented above)
