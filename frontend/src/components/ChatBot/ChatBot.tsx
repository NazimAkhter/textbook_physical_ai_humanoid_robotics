/**
 * ChatBot Component
 *
 * Main container component that orchestrates all ChatBot functionality.
 * Manages state, coordinates child components, and handles user interactions.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-001, FR-002, FR-003, FR-008, FR-017
 */

import React, { useState, useCallback, useRef, useEffect } from 'react';
import { ChatToggleButton } from './ChatToggleButton';
import { ChatPanel } from './ChatPanel';
import { useChatState } from '../../hooks/useChatState';
import { mockChatAPI } from '../../services/mockChatService';
import { logError } from '../../utils/errorLogger';
import type { Message } from '../../types/message';

// Import theme variables for CSS custom properties
import './styles/theme-variables.css';

export interface ChatBotProps {
  /** Optional initial open state */
  initialOpen?: boolean;
  /** Optional welcome title */
  welcomeTitle?: string;
  /** Optional welcome message */
  welcomeMessage?: string;
}

/**
 * ChatBot - Main chat interface component
 *
 * Features:
 * - Toggle button to open/close panel (FR-001)
 * - Persistent conversation state (FR-008)
 * - Message sending with loading states (FR-003)
 * - Session storage persistence
 *
 * Usage:
 * ```tsx
 * <ChatBot />
 * // or with custom welcome
 * <ChatBot
 *   welcomeTitle="Ask About Robotics"
 *   welcomeMessage="I can help you learn about humanoid robots."
 * />
 * ```
 */
export function ChatBot({
  initialOpen = false,
  welcomeTitle = 'AI Learning Assistant',
  welcomeMessage = "I'm here to help you understand Physical AI and humanoid robotics concepts. Ask me anything about the topics covered in this book!",
}: ChatBotProps): React.ReactElement {
  // Panel visibility state
  const [isOpen, setIsOpen] = useState(initialOpen);

  // Chat state management (messages, loading, storage)
  const {
    messages,
    isLoading,
    isStorageFallback,
    addMessage,
    clearMessages,
    setLoading,
  } = useChatState();

  // Input value state
  const [inputValue, setInputValue] = useState('');

  // Ref for input focus management
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Focus input when panel opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      // Small delay to ensure panel animation completes
      const timer = setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
      return () => clearTimeout(timer);
    }
  }, [isOpen]);

  // Toggle panel open/close
  const handleToggle = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  // Close panel
  const handleClose = useCallback(() => {
    setIsOpen(false);
  }, []);

  // Clear conversation
  const handleClear = useCallback(() => {
    clearMessages();
  }, [clearMessages]);

  // Send message
  const handleSendMessage = useCallback(
    async (content: string) => {
      if (!content.trim() || isLoading) {
        return;
      }

      // Clear input immediately
      setInputValue('');

      // Create and add user message
      const userMessage: Message = {
        id: `user-${Date.now()}`,
        role: 'user',
        content: content.trim(),
        timestamp: new Date(),
      };
      addMessage(userMessage);

      // Set loading state
      setLoading(true);

      try {
        // Send to mock service and get response
        const response = await mockChatAPI.sendMessage({
          message: content.trim(),
          conversationHistory: messages.map((m) => ({
            role: m.role,
            content: m.content,
            timestamp: m.timestamp.toISOString(),
          })),
        });

        // Create and add assistant message
        const assistantMessage: Message = {
          id: response.messageId,
          role: 'assistant',
          content: response.reply,
          timestamp: new Date(response.timestamp),
        };
        addMessage(assistantMessage);
      } catch (error) {
        // Log error (privacy-safe)
        logError(error instanceof Error ? error : new Error('Failed to send message'), {
          component: 'ChatBot',
          action: 'sendMessage',
        });

        // Add error message to chat
        const errorMessage: Message = {
          id: `error-${Date.now()}`,
          role: 'assistant',
          content:
            "I'm sorry, I encountered an error processing your request. Please try again.",
          timestamp: new Date(),
        };
        addMessage(errorMessage);
      } finally {
        setLoading(false);
      }
    },
    [isLoading, addMessage, setLoading]
  );

  return (
    <>
      {/* Toggle Button (FR-001) */}
      <ChatToggleButton
        isOpen={isOpen}
        onClick={handleToggle}
        ariaLabel={isOpen ? 'Close chat assistant' : 'Open chat assistant'}
      />

      {/* Chat Panel (FR-002) */}
      {isOpen && (
        <ChatPanel
          isOpen={isOpen}
          onClose={handleClose}
          messages={messages}
          isLoading={isLoading}
          inputValue={inputValue}
          onInputChange={setInputValue}
          onSendMessage={handleSendMessage}
          onClearConversation={handleClear}
          storageType={isStorageFallback ? 'memory' : 'session'}
          inputRef={inputRef}
          welcomeTitle={welcomeTitle}
          welcomeMessage={welcomeMessage}
        />
      )}
    </>
  );
}

export default ChatBot;
