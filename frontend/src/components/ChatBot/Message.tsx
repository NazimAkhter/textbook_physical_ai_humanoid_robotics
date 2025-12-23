/**
 * Message Component
 *
 * Individual message bubble with markdown rendering.
 * Distinguishes between user and assistant messages.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-005, FR-006, FR-007
 */

import React, { memo } from 'react';
import styles from './styles/Message.module.css';
import { MarkdownRenderer } from '../../utils/markdown';
import type { Message as MessageType } from '../../types/message';

export interface MessageProps {
  /** The message to display */
  message: MessageType;
  /** Additional CSS class names */
  className?: string;
}

/**
 * Format timestamp for display
 */
function formatTime(date: Date): string {
  return date.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit',
  });
}

/**
 * Message - Individual chat message bubble
 *
 * Features:
 * - Distinct styling for user vs assistant (FR-005)
 * - Markdown rendering (FR-006)
 * - Syntax highlighting for code (FR-007)
 *
 * Uses React.memo for performance optimization.
 *
 * Usage:
 * ```tsx
 * <Message message={msg} />
 * ```
 */
export const Message = memo(function Message({
  message,
  className = '',
}: MessageProps): React.ReactElement {
  const isUser = message.role === 'user';
  const isLoading = message.loadingState === true;

  return (
    <div
      className={`
        ${styles.messageWrapper}
        ${isUser ? styles.userWrapper : styles.assistantWrapper}
        ${className}
      `.trim()}
    >
      {/* Avatar */}
      <div
        className={`${styles.avatar} ${isUser ? styles.userAvatar : styles.assistantAvatar}`}
        aria-hidden="true"
      >
        {isUser ? (
          // User icon
          <svg
            width="16"
            height="16"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
            <circle cx="12" cy="7" r="4" />
          </svg>
        ) : (
          // Bot icon
          <svg
            width="16"
            height="16"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <rect x="3" y="11" width="18" height="10" rx="2" />
            <circle cx="12" cy="5" r="2" />
            <path d="M12 7v4" />
            <line x1="8" y1="16" x2="8" y2="16" />
            <line x1="16" y1="16" x2="16" y2="16" />
          </svg>
        )}
      </div>

      {/* Message bubble */}
      <div
        className={`
          ${styles.messageBubble}
          ${isUser ? styles.userBubble : styles.assistantBubble}
          ${isLoading ? styles.loadingBubble : ''}
        `.trim()}
      >
        {/* Role label for screen readers */}
        <span className={styles.srOnly}>
          {isUser ? 'You' : 'Assistant'}:
        </span>

        {/* Message content */}
        {isLoading ? (
          <div className={styles.loadingDots}>
            <span className={styles.dot} />
            <span className={styles.dot} />
            <span className={styles.dot} />
          </div>
        ) : isUser ? (
          // User messages: plain text (no markdown processing for security)
          <p className={styles.userContent}>{message.content}</p>
        ) : (
          // Assistant messages: markdown rendered (FR-006, FR-007)
          <div className={styles.assistantContent}>
            <MarkdownRenderer content={message.content} />
          </div>
        )}

        {/* Timestamp */}
        {!isLoading && (
          <time
            className={styles.timestamp}
            dateTime={message.timestamp.toISOString()}
          >
            {formatTime(message.timestamp)}
          </time>
        )}
      </div>
    </div>
  );
});

export default Message;
