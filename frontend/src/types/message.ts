/**
 * Message Type Definitions
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: data-model.md
 */

/**
 * Represents a single chat message from either the user or the chatbot assistant.
 */
export interface Message {
  /** Unique identifier, format: `msg-{timestamp}` */
  id: string;

  /** Message content (supports markdown) */
  content: string;

  /** Role of the message sender */
  role: 'user' | 'assistant';

  /** When message was created */
  timestamp: Date;

  /** True if this is a pending assistant response (loading state) */
  loadingState?: boolean;
}

/**
 * Chat message format for API requests/responses
 * Uses ISO 8601 string timestamps for serialization
 */
export interface ChatMessage {
  /** Role of the message sender */
  role: 'user' | 'assistant';

  /** Message content (supports markdown) */
  content: string;

  /** ISO 8601 timestamp when message was created */
  timestamp: string;
}

/**
 * Validate message content (FR-015: empty/whitespace validation)
 * @param content Message content to validate
 * @returns Validation result with optional error message
 */
export function validateMessageContent(content: string): {
  valid: boolean;
  error?: string;
} {
  if (!content || content.trim().length === 0) {
    return { valid: false, error: 'Message cannot be empty' };
  }

  if (content.length > 1000) {
    return { valid: false, error: 'Message too long (max 1000 characters)' };
  }

  return { valid: true };
}

/**
 * Create a user message with auto-generated ID and timestamp
 * @param content Message content
 * @returns Complete Message object
 */
export function createUserMessage(content: string): Message {
  return {
    id: `msg-${Date.now()}`,
    content,
    role: 'user',
    timestamp: new Date(),
  };
}

/**
 * Create a loading placeholder message for assistant response
 * @returns Loading state Message object
 */
export function createLoadingMessage(): Message {
  return {
    id: `msg-loading-${Date.now()}`,
    content: '',
    role: 'assistant',
    timestamp: new Date(),
    loadingState: true,
  };
}
