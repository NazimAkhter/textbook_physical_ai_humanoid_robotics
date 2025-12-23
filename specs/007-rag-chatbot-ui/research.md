# Research: RAG Chatbot UI Technology Decisions

**Feature**: 007-rag-chatbot-ui
**Date**: 2025-12-23
**Purpose**: Resolve NEEDS CLARIFICATION items from plan.md Technical Context

---

## Research Task 1: Markdown Rendering Library Selection

### Question
Which markdown library best supports Docusaurus integration and syntax highlighting for chat responses?

### Options Evaluated

1. **react-markdown** + remark/rehype plugins
   - Bundle size: ~35KB (with rehype-highlight)
   - Prism.js compatibility: Via rehype-prism or rehype-highlight
   - XSS safety: Built-in HTML sanitization, safe by default
   - React 18 support: ✅ Full support
   - Docusaurus integration: Works alongside Docusaurus MDX (no conflicts)

2. **Docusaurus built-in MDX components** (MDXContent)
   - Bundle size: 0KB (already in bundle)
   - Prism.js compatibility: ✅ Uses same Prism themes as docs
   - XSS safety: ✅ Inherits Docusaurus sanitization
   - React 18 support: ✅ (part of Docusaurus 3.x)
   - Docusaurus integration: ✅ Perfect integration, zero config

3. **marked** + DOMPurify
   - Bundle size: ~50KB combined
   - Prism.js compatibility: Manual integration required
   - XSS safety: Requires DOMPurify post-processing
   - React 18 support: ⚠️ Returns HTML string, needs dangerouslySetInnerHTML
   - Docusaurus integration: Separate rendering pipeline

### Decision: **react-markdown**

**Rationale**:
- **Bundle size**: 35KB fits within 50KB budget (SC-003)
- **Safety**: Built-in sanitization reduces XSS risk (FR-030 privacy requirement)
- **Prism.js integration**: `rehype-prism-plus` plugin provides syntax highlighting consistent with Docusaurus
- **React-native**: Returns React elements, not HTML strings (better performance, security)
- **Ecosystem**: Large plugin ecosystem (rehype-slug for headings, remark-gfm for GitHub-flavored markdown)

**Rejected Alternatives**:
- **Docusaurus MDX**: Rejected because MDXContent is designed for static content compilation, not dynamic runtime rendering. Chat responses arrive asynchronously and can't leverage MDX's build-time optimizations. Additionally, MDX allows executable JS which is inappropriate for user-generated content.
- **marked + DOMPurify**: Rejected due to larger bundle size (50KB exceeds budget), manual sanitization complexity, and dangerouslySetInnerHTML security concerns.

### Implementation Notes
```javascript
// Install dependencies
npm install react-markdown rehype-prism-plus remark-gfm

// Usage in Message.tsx
import ReactMarkdown from 'react-markdown';
import rehypePrism from 'rehype-prism-plus';
import remarkGfm from 'remark-gfm';

<ReactMarkdown
  remarkPlugins={[remarkGfm]}
  rehypePlugins={[rehypePrism]}
>
  {message.content}
</ReactMarkdown>
```

---

## Research Task 2: Automated Testing Strategy

### Question
Should we adopt automated testing (Jest, React Testing Library, Playwright) or rely on manual acceptance testing?

### Evaluation Criteria
- **ROI**: Testing value vs implementation effort
- **CI/CD integration**: Ease of automation in GitHub Actions/Vercel
- **Coverage**: Ability to validate responsive design, accessibility, theme switching

### Analysis

**Pros of Automated Testing**:
- Regression prevention (FR changes don't break existing functionality)
- Accessibility validation (WCAG 2.1 AA compliance - SC-007)
- Faster validation cycles (no manual re-testing for every change)
- CI/CD integration enables pre-merge checks

**Cons of Automated Testing**:
- Setup effort (Jest, RTL, testing-library/user-event configuration)
- Maintenance burden (tests need updates when UI changes)
- Visual testing limitations (responsive design requires viewport simulation)
- Theme testing complexity (mocking Docusaurus theme context)

**Manual Testing Tradeoffs**:
- Lower initial effort (can start implementation faster)
- Better for visual QA (responsive design, theme aesthetics)
- Higher long-term cost (repetitive manual validation)
- No CI/CD automation (relies on human diligence)

### Decision: **Manual Testing for MVP, Automated Testing for Future Enhancement**

**Rationale**:
- **Scope**: Spec defines 34 FRs, 8 SCs, 4 user stories - comprehensive manual test plan already documented in plan.md
- **Timeline**: User requirement states "Complete within 1-2 weeks" - automated testing setup would consume ~20-30% of timeline
- **ROI**: UI-only feature with mock backend has limited regression risk during initial development
- **Validation**: Manual testing better suited for responsive design validation (actual device testing recommended)

**Manual Testing Strategy** (documented in plan.md Validation section):
- UI functional testing (toggle, send, render, clear)
- Responsive testing (desktop, tablet, mobile viewports)
- Theme testing (light/dark mode switching)
- Accessibility testing (keyboard navigation, screen reader, ARIA)
- Performance testing (bundle size, interaction latency, scroll performance)

**Future Enhancement Path**:
- Post-MVP: Add Jest + React Testing Library for component unit tests
- Focus areas: useChatState hook logic, storage fallback behavior, error boundary
- Playwright for E2E testing if chatbot becomes critical user path
- Accessibility automation via axe-core/jest-axe

---

## Research Task 3: Docusaurus Integration Pattern

### Question
Best practice for injecting global React component (ChatBot) into Docusaurus site?

### Options Evaluated

1. **Swizzle Root.tsx** (Theme Component Override)
   - **How**: `npx docusaurus swizzle @docusaurus/theme-classic Root --wrap`
   - **Pros**: Official Docusaurus pattern, survives upgrades (wrapper mode), global injection
   - **Cons**: Requires understanding of Docusaurus theme architecture
   - **Stability**: ✅ Stable API across Docusaurus versions

2. **Docusaurus Plugin Architecture**
   - **How**: Create custom plugin with `injectHtmlTags` lifecycle hook
   - **Pros**: Clean separation from theme, can bundle ChatBot as npm package
   - **Cons**: More boilerplate, plugin config complexity, async loading challenges
   - **Stability**: ✅ Stable but overkill for single component

3. **Direct DOM Injection** (componentDidMount)
   - **How**: Add script in `docusaurus.config.js` to inject React component via `ReactDOM.render`
   - **Pros**: No swizzling required
   - **Cons**: ⚠️ Fragile (DOM structure changes), hydration issues, not idiomatic React
   - **Stability**: ❌ Not recommended by Docusaurus team

### Decision: **Swizzle Root.tsx (Wrapper Mode)**

**Rationale**:
- **Official pattern**: Documented in Docusaurus guides (https://docusaurus.io/docs/swizzling)
- **Wrapper mode safety**: `--wrap` flag creates wrapper that calls original component, minimizing breakage risk
- **Global availability**: Root.tsx wraps all pages, ensuring ChatBot accessible everywhere (FR-001)
- **Upgrade stability**: Wrapper mode survives Docusaurus upgrades (original Root.tsx implementation can change without breaking wrapper)
- **React context access**: ChatBot can access Docusaurus theme context (useColorMode for dark/light mode)

**Implementation Steps**:
```bash
# Swizzle Root component in wrapper mode
npx docusaurus swizzle @docusaurus/theme-classic Root --wrap

# Result: Creates src/theme/Root.tsx
```

```typescript
// src/theme/Root.tsx (Swizzled)
import React from 'react';
import Root from '@theme-original/Root';
import ChatBot from '@site/frontend/src/components/ChatBot/ChatBot';

export default function RootWrapper(props) {
  return (
    <>
      <Root {...props} />
      <ChatBot />
    </>
  );
}
```

**Rejected Alternatives**:
- **Plugin**: Rejected due to unnecessary complexity for single global component
- **DOM Injection**: Rejected as anti-pattern (hydration issues, not idiomatic React, fragile)

---

## Research Task 4: Session Storage Fallback Implementation

### Question
How to reliably detect session storage unavailability and switch to in-memory storage?

### Private Browsing Detection Patterns

**Challenge**: Session storage exists in private browsing but throws exceptions on setItem() due to quota restrictions.

**Detection Strategies**:

1. **Try-catch on setItem()** (Recommended)
```javascript
function isSessionStorageAvailable() {
  try {
    const testKey = '__storage_test__';
    sessionStorage.setItem(testKey, 'test');
    sessionStorage.removeItem(testKey);
    return true;
  } catch (e) {
    return false;
  }
}
```

2. **Storage quota check** (QuotaExceededError detection)
```javascript
function detectStorageQuotaExceeded() {
  try {
    sessionStorage.setItem('test', 'data');
  } catch (e) {
    if (e.name === 'QuotaExceededError') {
      return true; // Quota exceeded
    }
    return false; // Other error (e.g., SecurityError in private browsing)
  }
}
```

3. **Feature detection + write test** (Combined approach)
```javascript
function hasSessionStorage() {
  if (typeof window === 'undefined' || !window.sessionStorage) {
    return false; // SSR or very old browser
  }

  try {
    const testKey = `__test_${Date.now()}__`;
    sessionStorage.setItem(testKey, 'test');
    sessionStorage.removeItem(testKey);
    return true;
  } catch (e) {
    console.warn('Session storage unavailable:', e.name);
    return false; // Private browsing or quota exceeded
  }
}
```

### Decision: **Try-catch on setItem() with Feature Detection**

**Rationale**:
- **Reliability**: Writing test value is only way to detect quota/privacy restrictions
- **Graceful degradation**: Matches FR-025, FR-026 requirements
- **User feedback**: FR-027 requires subtle notification when fallback active

**Implementation Pattern**:
```typescript
// services/storageService.ts
class StorageService {
  private storageType: 'session' | 'memory' = 'session';
  private memoryStore: Map<string, any> = new Map();

  constructor() {
    this.storageType = this.detectStorageAvailability();
  }

  private detectStorageAvailability(): 'session' | 'memory' {
    try {
      const testKey = '__chatbot_storage_test__';
      sessionStorage.setItem(testKey, 'test');
      sessionStorage.removeItem(testKey);
      return 'session';
    } catch (e) {
      console.warn('[ChatBot] Session storage unavailable, using in-memory fallback');
      return 'memory';
    }
  }

  getConversation(): ConversationState | null {
    if (this.storageType === 'session') {
      const stored = sessionStorage.getItem('chatbot_conversation');
      return stored ? JSON.parse(stored) : null;
    } else {
      return this.memoryStore.get('chatbot_conversation') || null;
    }
  }

  saveConversation(state: ConversationState): void {
    if (this.storageType === 'session') {
      try {
        sessionStorage.setItem('chatbot_conversation', JSON.stringify(state));
      } catch (e) {
        // Quota exceeded mid-session, switch to memory
        console.warn('[ChatBot] Switching to in-memory storage due to quota');
        this.storageType = 'memory';
        this.memoryStore.set('chatbot_conversation', state);
      }
    } else {
      this.memoryStore.set('chatbot_conversation', state);
    }
  }

  clearConversation(): void {
    if (this.storageType === 'session') {
      sessionStorage.removeItem('chatbot_conversation');
    } else {
      this.memoryStore.delete('chatbot_conversation');
    }
  }

  getStorageType(): 'session' | 'memory' {
    return this.storageType;
  }
}
```

**Notification Strategy** (FR-027):
```typescript
// In ChatBot.tsx or ChatPanel.tsx
{storageType === 'memory' && (
  <div className="storage-fallback-notice">
    ℹ️ Conversation history won't persist across pages (private browsing detected)
  </div>
)}
```

---

## Research Task 5: Error Boundary Best Practices

### Question
How to implement React error boundary for chat component with reload functionality?

### React 18 Error Boundary Pattern

**Key Changes in React 18**:
- Error boundaries still class components (no hooks equivalent yet)
- `componentDidCatch` for side effects (logging)
- `static getDerivedStateFromError` for state updates (render fallback)

**Implementation**:
```typescript
// components/ChatBot/ErrorBoundary.tsx
import React, { Component, ReactNode } from 'react';

interface Props {
  children: ReactNode;
  onError?: (error: Error, errorInfo: React.ErrorInfo) => void;
}

interface State {
  hasError: boolean;
  error: Error | null;
}

class ChatBotErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: Error): State {
    // Update state to render fallback UI
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    // Log error to console (development) or error service (production)
    console.error('[ChatBot Error Boundary]', error, errorInfo);

    // Optional callback for external error tracking (FR-029)
    if (this.props.onError) {
      this.props.onError(error, errorInfo);
    }
  }

  handleReload = () => {
    // Reset error state to retry rendering
    this.setState({ hasError: false, error: null });
  };

  render() {
    if (this.state.hasError) {
      // Fallback UI with reload option
      return (
        <div className="chatbot-error-fallback">
          <h3>⚠️ Chatbot Error</h3>
          <p>Something went wrong with the chat interface.</p>
          <button onClick={this.handleReload}>
            Reload Chat
          </button>
          {process.env.NODE_ENV === 'development' && (
            <details>
              <summary>Error Details</summary>
              <pre>{this.state.error?.message}</pre>
            </details>
          )}
        </div>
      );
    }

    return this.props.children;
  }
}

export default ChatBotErrorBoundary;
```

### Decision: **Class Component Error Boundary with Reload**

**Rationale**:
- **React 18 requirement**: No functional component alternative for error boundaries yet
- **User recovery**: Reload button allows users to recover without page refresh (better UX than crash)
- **Logging integration**: `onError` callback supports FR-029 (optional production error tracking)
- **Privacy**: FR-030 requires no sensitive data in logs - error message sanitized in production

**Usage**:
```typescript
// ChatBot.tsx
import ErrorBoundary from './ErrorBoundary';

function ChatBot() {
  const errorCallback = (error: Error, errorInfo: React.ErrorInfo) => {
    // Optional: Send to Sentry, LogRocket, etc. if configured
    window.chatbotErrorHandler?.(error, errorInfo);
  };

  return (
    <ErrorBoundary onError={errorCallback}>
      <ChatToggleButton ... />
      <ChatPanel ... />
    </ErrorBoundary>
  );
}
```

---

## Summary of Decisions

| Research Task | Decision | Rationale |
|---------------|----------|-----------|
| **Markdown Library** | react-markdown + rehype-prism-plus | 35KB bundle, safe by default, Prism.js compatible, React 18 support |
| **Testing Strategy** | Manual testing for MVP, automated future | Faster MVP delivery, comprehensive manual plan, automation post-launch |
| **Docusaurus Integration** | Swizzle Root.tsx (wrapper mode) | Official pattern, stable across upgrades, global component access |
| **Storage Fallback** | Try-catch setItem() with in-memory Map | Reliable quota detection, graceful degradation, FR-025/026 compliance |
| **Error Boundary** | Class component with reload button | React 18 best practice, user recovery, logging integration (FR-029) |

---

## Technical Context Updates (Resolved NEEDS CLARIFICATION)

**Before**:
- Markdown library: NEEDS CLARIFICATION
- Testing strategy: NEEDS CLARIFICATION

**After**:
- **Markdown Library**: react-markdown v9.x + rehype-prism-plus (syntax highlighting) + remark-gfm (GitHub-flavored markdown)
- **Testing Strategy**: Manual UI/responsive/theme/accessibility/performance testing per plan.md validation checklist; automated testing deferred to post-MVP enhancement

---

**Research Status**: ✅ Complete
**All NEEDS CLARIFICATION Resolved**: Yes
**Ready for Phase 1**: Yes (data-model.md, contracts/, quickstart.md generation)
