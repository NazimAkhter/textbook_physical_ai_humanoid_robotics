# Data Model: RAG Chatbot UI

**Feature**: 007-rag-chatbot-ui
**Date**: 2025-12-23
**Purpose**: Define entities, state management, and data flow for chatbot UI components

---

## Entities

### 1. Message

Represents a single chat message from either the user or the chatbot assistant.

**Fields**:
- `id`: string (unique identifier, format: `msg-{timestamp}`)
- `content`: string (message text, supports markdown)
- `role`: `'user' | 'assistant'` (message sender)
- `timestamp`: Date (when message was created)
- `loadingState`: boolean (true if this is a pending assistant response)

**Validation Rules**:
- `content`: MUST NOT be empty or whitespace-only (FR-015)
- `role`: MUST be enum value (`user` or `assistant`)
- `timestamp`: MUST be valid Date object
- `loadingState`: Only applicable for `role === 'assistant'`

**Relationships**:
- Part of `messages[]` array in Conversation State
- Display order: chronological (oldest to newest)

**Example**:
```typescript
interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  loadingState?: boolean;
}

// User message example
const userMessage: Message = {
  id: 'msg-1703347200000',
  content: 'What is VLA?',
  role: 'user',
  timestamp: new Date('2025-12-23T10:00:00Z'),
};

// Assistant message example
const assistantMessage: Message = {
  id: 'msg-1703347201500',
  content: '**Vision-Language-Action (VLA)** systems combine...',
  role: 'assistant',
  timestamp: new Date('2025-12-23T10:00:01.5Z'),
};

// Loading state (pending response)
const loadingMessage: Message = {
  id: 'msg-1703347202000',
  content: '',
  role: 'assistant',
  timestamp: new Date('2025-12-23T10:00:02Z'),
  loadingState: true,
};
```

---

### 2. Conversation State

Represents the current chat session's data and UI state.

**Fields**:
- `messages`: Message[] (conversation history, ordered chronologically)
- `inputValue`: string (current text in message input field)
- `isPanelOpen`: boolean (whether chat panel is visible)
- `isLoading`: boolean (whether waiting for assistant response)
- `storageType`: `'session' | 'memory'` (which storage backend is active)

**Lifecycle**:
1. **Creation**: Initialized on first chat toggle button click or on page load if conversation exists in storage
2. **Persistence**: Saved to session storage after every state change (or in-memory if storage unavailable)
3. **Restoration**: Loaded from session storage on page navigation (FR-021, FR-023)
4. **Reset**: Cleared on user "Clear conversation" action (FR-013) or browser session end

**State Transitions**:
```
Initial State → Panel Closed
    ↓ (user clicks toggle)
Panel Open (Empty State: Welcome Message)
    ↓ (user types and sends message)
Panel Open + isLoading=true (User message added, waiting for response)
    ↓ (response received)
Panel Open + isLoading=false (Assistant message added)
    ↓ (user navigates to different page)
Panel Closed (Conversation persists in storage)
    ↓ (user clicks toggle on new page)
Panel Open (Conversation restored from storage)
    ↓ (user clicks Clear conversation)
Panel Open (Empty State: Welcome Message, storage cleared)
```

**Example**:
```typescript
interface ConversationState {
  messages: Message[];
  inputValue: string;
  isPanelOpen: boolean;
  isLoading: boolean;
  storageType: 'session' | 'memory';
}

// Initial state (empty conversation)
const initialState: ConversationState = {
  messages: [],
  inputValue: '',
  isPanelOpen: false,
  isLoading: false,
  storageType: 'session', // or 'memory' if storage unavailable
};

// Active conversation state
const activeState: ConversationState = {
  messages: [
    { id: 'msg-1', content: 'What is SLAM?', role: 'user', timestamp: new Date() },
    { id: 'msg-2', content: '**SLAM** stands for...', role: 'assistant', timestamp: new Date() },
  ],
  inputValue: 'How does it work?',
  isPanelOpen: true,
  isLoading: false,
  storageType: 'session',
};
```

---

### 3. Storage Abstraction

Interface for conversation persistence with session storage and in-memory fallback.

**Interface**:
```typescript
interface IStorageService {
  getConversation(): ConversationState | null;
  saveConversation(state: ConversationState): void;
  clearConversation(): void;
  getStorageType(): 'session' | 'memory';
}
```

**Implementations**:

**SessionStorageAdapter**:
```typescript
class SessionStorageAdapter implements IStorageService {
  private readonly STORAGE_KEY = 'chatbot_conversation';

  getConversation(): ConversationState | null {
    try {
      const stored = sessionStorage.getItem(this.STORAGE_KEY);
      if (!stored) return null;

      const parsed = JSON.parse(stored);
      // Reconstruct Date objects (JSON.parse converts to strings)
      parsed.messages = parsed.messages.map(msg => ({
        ...msg,
        timestamp: new Date(msg.timestamp),
      }));
      return parsed;
    } catch (e) {
      console.error('[Storage] Failed to retrieve conversation', e);
      return null;
    }
  }

  saveConversation(state: ConversationState): void {
    try {
      sessionStorage.setItem(this.STORAGE_KEY, JSON.stringify(state));
    } catch (e) {
      // Quota exceeded or storage unavailable
      throw new StorageUnavailableError('Session storage write failed');
    }
  }

  clearConversation(): void {
    sessionStorage.removeItem(this.STORAGE_KEY);
  }

  getStorageType(): 'session' {
    return 'session';
  }
}
```

**InMemoryStorageAdapter**:
```typescript
class InMemoryStorageAdapter implements IStorageService {
  private store: Map<string, ConversationState> = new Map();
  private readonly STORAGE_KEY = 'chatbot_conversation';

  getConversation(): ConversationState | null {
    return this.store.get(this.STORAGE_KEY) || null;
  }

  saveConversation(state: ConversationState): void {
    this.store.set(this.STORAGE_KEY, state);
  }

  clearConversation(): void {
    this.store.delete(this.STORAGE_KEY);
  }

  getStorageType(): 'memory' {
    return 'memory';
  }
}
```

**Fallback Logic** (FR-025, FR-026):
```typescript
// services/storageService.ts
function createStorageService(): IStorageService {
  try {
    // Test session storage availability
    const testKey = '__chatbot_test__';
    sessionStorage.setItem(testKey, 'test');
    sessionStorage.removeItem(testKey);
    return new SessionStorageAdapter();
  } catch (e) {
    console.warn('[Storage] Session storage unavailable, using in-memory fallback');
    return new InMemoryStorageAdapter();
  }
}

export const storageService = createStorageService();
```

---

## State Management

### useChatState Hook

Custom React hook for managing conversation state with storage persistence.

**Signature**:
```typescript
interface UseChatStateReturn {
  messages: Message[];
  inputValue: string;
  isPanelOpen: boolean;
  isLoading: boolean;
  storageType: 'session' | 'memory';

  // Actions
  openPanel: () => void;
  closePanel: () => void;
  setInputValue: (value: string) => void;
  sendMessage: (content: string) => Promise<void>;
  clearConversation: () => void;
}

function useChatState(): UseChatStateReturn;
```

**Implementation Pattern**:
```typescript
import { useState, useEffect, useCallback } from 'react';
import { storageService } from '../services/storageService';
import { mockChatAPI } from '../services/mockChatService';

function useChatState(): UseChatStateReturn {
  const [state, setState] = useState<ConversationState>(() => {
    // Initialize from storage or default state
    const stored = storageService.getConversation();
    return stored || {
      messages: [],
      inputValue: '',
      isPanelOpen: false,
      isLoading: false,
      storageType: storageService.getStorageType(),
    };
  });

  // Persist state changes to storage
  useEffect(() => {
    storageService.saveConversation(state);
  }, [state]);

  const openPanel = useCallback(() => {
    setState(prev => ({ ...prev, isPanelOpen: true }));
  }, []);

  const closePanel = useCallback(() => {
    setState(prev => ({ ...prev, isPanelOpen: false }));
  }, []);

  const setInputValue = useCallback((value: string) => {
    setState(prev => ({ ...prev, inputValue: value }));
  }, []);

  const sendMessage = useCallback(async (content: string) => {
    // Add user message
    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      content,
      role: 'user',
      timestamp: new Date(),
    };

    setState(prev => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      inputValue: '',
      isLoading: true,
    }));

    try {
      // Call mock API (future: replace with real RAG backend)
      const response = await mockChatAPI.sendMessage({
        message: content,
        conversationHistory: state.messages.map(msg => ({
          role: msg.role,
          content: msg.content,
          timestamp: msg.timestamp.toISOString(),
        })),
      });

      // Add assistant response
      const assistantMessage: Message = {
        id: response.messageId,
        content: response.reply,
        role: 'assistant',
        timestamp: new Date(response.timestamp),
      };

      setState(prev => ({
        ...prev,
        messages: [...prev.messages, assistantMessage],
        isLoading: false,
      }));
    } catch (error) {
      console.error('[Chat] Failed to send message', error);
      setState(prev => ({ ...prev, isLoading: false }));
      // TODO: Show error message to user
    }
  }, [state.messages]);

  const clearConversation = useCallback(() => {
    setState({
      messages: [],
      inputValue: '',
      isPanelOpen: true, // Keep panel open after clear
      isLoading: false,
      storageType: storageService.getStorageType(),
    });
    storageService.clearConversation();
  }, []);

  return {
    messages: state.messages,
    inputValue: state.inputValue,
    isPanelOpen: state.isPanelOpen,
    isLoading: state.isLoading,
    storageType: state.storageType,
    openPanel,
    closePanel,
    setInputValue,
    sendMessage,
    clearConversation,
  };
}
```

---

## Data Flow

### Message Submission Flow

1. **User Input** → MessageInput component
   - User types message, presses Enter
   - Validation: Empty/whitespace check (FR-015)
   - Triggers: `onSendMessage(inputValue)`

2. **State Update** → useChatState hook
   - Add user message to `messages[]`
   - Clear `inputValue`
   - Set `isLoading = true`
   - Persist state to storage

3. **API Call** → mockChatService
   - Send message with conversation history
   - Mock delay: 500-1500ms
   - Return canned response or keyword-based reply

4. **Response Handling** → useChatState hook
   - Add assistant message to `messages[]`
   - Set `isLoading = false`
   - Persist updated state to storage

5. **UI Update** → MessageList component
   - Re-render with new messages
   - Auto-scroll to bottom (FR-009)
   - Remove loading indicator

### Storage Persistence Flow

1. **State Change** → useState setter in useChatState
2. **Effect Trigger** → useEffect dependency on state
3. **Storage Write** → storageService.saveConversation(state)
4. **Session Storage** → sessionStorage.setItem() or Map.set() (fallback)

### Page Navigation Flow (FR-021, FR-022, FR-023)

1. **Before Navigation** → Panel closes (FR-022)
   - `isPanelOpen` set to `false` in state
   - Conversation persisted to storage automatically

2. **Page Load** → New page renders
   - Root.tsx renders ChatBot component
   - useChatState initializes from storage
   - Conversation restored from sessionStorage.getItem()

3. **Panel Reopen** → User clicks toggle button
   - Conversation history displays immediately
   - No loss of context across pages

---

## Validation Rules

### Message Validation

```typescript
function validateMessage(content: string): { valid: boolean; error?: string } {
  if (!content || content.trim().length === 0) {
    return { valid: false, error: 'Message cannot be empty' };
  }

  if (content.length > 1000) {
    return { valid: false, error: 'Message too long (max 1000 characters)' };
  }

  return { valid: true };
}
```

### State Validation

```typescript
function isValidConversationState(state: unknown): state is ConversationState {
  if (typeof state !== 'object' || state === null) return false;

  const s = state as Partial<ConversationState>;

  return (
    Array.isArray(s.messages) &&
    typeof s.inputValue === 'string' &&
    typeof s.isPanelOpen === 'boolean' &&
    typeof s.isLoading === 'boolean' &&
    (s.storageType === 'session' || s.storageType === 'memory')
  );
}
```

---

## Performance Considerations

### Message List Optimization (SC-005: 20+ messages)

**Problem**: Large message arrays can cause scroll lag during re-renders.

**Solutions**:
1. **React.memo** on Message component (prevent unnecessary re-renders)
2. **Virtual scrolling** (if >100 messages, use react-window)
3. **Pagination** (load older messages on scroll up)

**Current Scope**: Manual testing with 20+ messages, no virtual scrolling for MVP (spec only requires 20 messages without lag).

### Storage Size Management

**Problem**: Session storage has 5-10MB limit; long conversations may exceed quota.

**Solutions**:
1. **Quota monitoring**: Track storage size after each save
2. **Message truncation**: Keep only last N messages (e.g., 50)
3. **Graceful degradation**: Switch to in-memory if quota exceeded mid-session (already implemented in research.md)

**Current Scope**: No explicit truncation for MVP; FR-026 handles quota exceeded by switching to memory storage.

---

**Data Model Status**: ✅ Complete
**Ready for Contract Generation**: Yes
**Next Step**: Create contracts/mock-api.ts
