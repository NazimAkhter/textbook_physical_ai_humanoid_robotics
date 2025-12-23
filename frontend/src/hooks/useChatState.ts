/**
 * useChatState Hook
 *
 * Custom React hook for managing conversation state with storage persistence.
 * Handles message sending, conversation history, and panel visibility.
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: data-model.md
 * Phase 4 Enhancements: T035-T039 (persistence, restoration)
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { storageService } from '../services/storageService';
import { mockChatAPI } from '../services/mockChatService';
import { logError, logChatWarning } from '../utils/errorLogger';
import type {
  ConversationState,
  UseChatStateReturn,
  SimplifiedChatStateReturn,
  StorageType,
} from '../types/chatState';
import type { Message } from '../types/message';
import { validateMessageContent, createUserMessage } from '../types/message';

/**
 * Initial conversation state (empty, panel closed)
 */
const createInitialState = (storageType: StorageType): ConversationState => ({
  messages: [],
  inputValue: '',
  isPanelOpen: false,
  isLoading: false,
  storageType,
});

/**
 * useChatState - Conversation state management hook
 *
 * Provides:
 * - Message list management
 * - Input field state
 * - Panel open/close state
 * - Loading state for async operations
 * - Automatic persistence to session storage (T038)
 * - Conversation restoration on init (T039, FR-023)
 *
 * Usage:
 * ```tsx
 * const {
 *   messages,
 *   inputValue,
 *   isPanelOpen,
 *   isLoading,
 *   storageType,
 *   openPanel,
 *   closePanel,
 *   togglePanel,
 *   setInputValue,
 *   sendMessage,
 *   clearConversation,
 * } = useChatState();
 * ```
 */
export function useChatState(): UseChatStateReturn & SimplifiedChatStateReturn {
  // Track if this is the initial mount
  const isInitialMount = useRef(true);

  // Initialize state from storage or defaults (T039: conversation restoration)
  const [state, setState] = useState<ConversationState>(() => {
    try {
      const stored = storageService.getConversation();
      if (stored) {
        // Always start with panel closed (FR-022: panel closes on navigation)
        return {
          ...stored,
          isPanelOpen: false,
          isLoading: false,
        };
      }
    } catch (e) {
      logError(e as Error, 'warn', {
        component: 'useChatState',
        action: 'initializeFromStorage',
      });
    }
    return createInitialState(storageService.getStorageType());
  });

  // Persist state changes to storage (T038: conversation history persistence)
  // Skip initial mount to avoid unnecessary writes
  useEffect(() => {
    if (isInitialMount.current) {
      isInitialMount.current = false;
      return;
    }

    try {
      // Only persist messages, not transient UI state
      const persistedState: ConversationState = {
        ...state,
        isPanelOpen: false, // Don't persist panel state
        isLoading: false, // Don't persist loading state
      };
      storageService.saveConversation(persistedState);
    } catch (e) {
      logError(e as Error, 'warn', {
        component: 'useChatState',
        action: 'persistState',
      });
    }
  }, [state.messages]); // Only persist on message changes

  // Action: Open panel
  const openPanel = useCallback(() => {
    setState(prev => ({ ...prev, isPanelOpen: true }));
  }, []);

  // Action: Close panel
  const closePanel = useCallback(() => {
    setState(prev => ({ ...prev, isPanelOpen: false }));
  }, []);

  // Action: Toggle panel
  const togglePanel = useCallback(() => {
    setState(prev => ({ ...prev, isPanelOpen: !prev.isPanelOpen }));
  }, []);

  // Action: Set input value
  const setInputValue = useCallback((value: string) => {
    setState(prev => ({ ...prev, inputValue: value }));
  }, []);

  // Action: Add a single message (simplified interface for ChatBot.tsx)
  const addMessage = useCallback((message: Message) => {
    setState(prev => ({
      ...prev,
      messages: [...prev.messages, message],
    }));
  }, []);

  // Action: Clear messages (simplified interface for ChatBot.tsx)
  const clearMessages = useCallback(() => {
    setState(prev => ({
      ...prev,
      messages: [],
    }));

    try {
      storageService.clearConversation();
    } catch (e) {
      logError(e as Error, 'warn', {
        component: 'useChatState',
        action: 'clearMessages',
      });
    }
  }, []);

  // Action: Set loading state (simplified interface for ChatBot.tsx)
  const setLoading = useCallback((loading: boolean) => {
    setState(prev => ({ ...prev, isLoading: loading }));
  }, []);

  // Action: Send message (full interface)
  const sendMessage = useCallback(async (content: string): Promise<void> => {
    // Validate message content (FR-015)
    const validation = validateMessageContent(content);
    if (!validation.valid) {
      logChatWarning(validation.error || 'Invalid message', {
        component: 'useChatState',
        action: 'sendMessage',
      });
      return;
    }

    // Create user message
    const userMessage = createUserMessage(content.trim());

    // Update state: add user message, clear input, set loading
    setState(prev => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      inputValue: '',
      isLoading: true,
    }));

    try {
      // Get current messages for conversation history
      // Note: Using callback form to get latest state
      const currentMessages = await new Promise<Message[]>(resolve => {
        setState(prev => {
          resolve(prev.messages);
          return prev;
        });
      });

      // Call chat API
      const response = await mockChatAPI.sendMessage({
        message: content.trim(),
        conversationHistory: currentMessages.map(msg => ({
          role: msg.role,
          content: msg.content,
          timestamp: msg.timestamp.toISOString(),
        })),
      });

      // Create assistant message from response
      const assistantMessage: Message = {
        id: response.messageId,
        content: response.reply,
        role: 'assistant',
        timestamp: new Date(response.timestamp),
      };

      // Update state: add assistant message, clear loading
      setState(prev => ({
        ...prev,
        messages: [...prev.messages, assistantMessage],
        isLoading: false,
      }));
    } catch (e) {
      logError(e as Error, 'error', {
        component: 'useChatState',
        action: 'sendMessage',
      });

      // Clear loading state on error
      setState(prev => ({ ...prev, isLoading: false }));

      // Re-throw to allow UI to handle error display
      throw e;
    }
  }, []);

  // Action: Clear conversation (full interface)
  const clearConversation = useCallback(() => {
    setState(prev => ({
      messages: [],
      inputValue: '',
      isPanelOpen: true, // Keep panel open after clear
      isLoading: false,
      storageType: prev.storageType,
    }));

    try {
      storageService.clearConversation();
    } catch (e) {
      logError(e as Error, 'warn', {
        component: 'useChatState',
        action: 'clearConversation',
      });
    }
  }, []);

  // Compute isStorageFallback for simplified interface
  const isStorageFallback = state.storageType === 'memory';

  return {
    // State (full interface)
    messages: state.messages,
    inputValue: state.inputValue,
    isPanelOpen: state.isPanelOpen,
    isLoading: state.isLoading,
    storageType: state.storageType,

    // Actions (full interface)
    openPanel,
    closePanel,
    togglePanel,
    setInputValue,
    sendMessage,
    clearConversation,

    // Simplified interface for ChatBot.tsx
    isStorageFallback,
    addMessage,
    clearMessages,
    setLoading,
  };
}

export default useChatState;
