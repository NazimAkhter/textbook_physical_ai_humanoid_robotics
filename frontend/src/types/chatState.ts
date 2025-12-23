/**
 * Conversation State Type Definitions
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: data-model.md
 */

import type { Message } from './message';

/**
 * Storage type indicator for conversation persistence
 */
export type StorageType = 'session' | 'memory';

/**
 * Represents the current chat session's data and UI state.
 *
 * Lifecycle:
 * 1. Creation: Initialized on first chat toggle or on page load if conversation exists
 * 2. Persistence: Saved to session storage after every state change
 * 3. Restoration: Loaded from session storage on page navigation
 * 4. Reset: Cleared on user "Clear conversation" action or browser session end
 */
export interface ConversationState {
  /** Conversation history, ordered chronologically (oldest to newest) */
  messages: Message[];

  /** Current text in message input field */
  inputValue: string;

  /** Whether chat panel is visible */
  isPanelOpen: boolean;

  /** Whether waiting for assistant response */
  isLoading: boolean;

  /** Which storage backend is active */
  storageType: StorageType;
}

/**
 * Initial conversation state (empty conversation, panel closed)
 */
export const initialConversationState: ConversationState = {
  messages: [],
  inputValue: '',
  isPanelOpen: false,
  isLoading: false,
  storageType: 'session',
};

/**
 * Actions available for modifying conversation state
 */
export interface ConversationActions {
  /** Open the chat panel */
  openPanel: () => void;

  /** Close the chat panel */
  closePanel: () => void;

  /** Toggle the chat panel open/closed */
  togglePanel: () => void;

  /** Set the input field value */
  setInputValue: (value: string) => void;

  /** Send a message and get response */
  sendMessage: (content: string) => Promise<void>;

  /** Clear conversation history and reset to welcome state */
  clearConversation: () => void;
}

/**
 * Combined state and actions for useChatState hook return type
 */
export interface UseChatStateReturn extends ConversationState, ConversationActions {}

/**
 * Simplified interface for ChatBot component usage
 * Provides addMessage/clearMessages/setLoading for manual control
 */
export interface SimplifiedChatStateReturn {
  /** Conversation messages */
  messages: Message[];

  /** Loading state */
  isLoading: boolean;

  /** Whether using memory fallback instead of session storage */
  isStorageFallback: boolean;

  /** Add a single message to conversation */
  addMessage: (message: Message) => void;

  /** Clear all messages */
  clearMessages: () => void;

  /** Set loading state */
  setLoading: (loading: boolean) => void;
}

/**
 * Validate conversation state structure (for storage restoration)
 * @param state Unknown state object from storage
 * @returns True if state is valid ConversationState
 */
export function isValidConversationState(state: unknown): state is ConversationState {
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
