/**
 * ChatHeader Component
 *
 * Header for the chat panel with title and close button.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-010, FR-013
 * Phase 7: T074 - React.memo optimization
 */

import React, { memo } from 'react';
import styles from './styles/ChatPanel.module.css';

export interface ChatHeaderProps {
  /** Callback when close button is clicked */
  onClose: () => void;
  /** Callback when clear button is clicked (FR-013) */
  onClear?: () => void;
  /** Whether to show the clear button */
  showClear?: boolean;
  /** Custom title (defaults to "Book Assistant") */
  title?: string;
}

/**
 * ChatHeader - Panel header with title and actions
 *
 * Features:
 * - Title display
 * - Close button (FR-010)
 * - Clear conversation button (FR-013, shown when messages exist)
 *
 * Usage:
 * ```tsx
 * <ChatHeader
 *   onClose={closePanel}
 *   onClear={clearConversation}
 *   showClear={messages.length > 0}
 * />
 * ```
 */
export const ChatHeader = memo(function ChatHeader({
  onClose,
  onClear,
  showClear = false,
  title = 'Book Assistant',
}: ChatHeaderProps): React.ReactElement {
  return (
    <header className={styles.header}>
      <h2 id="chatbot-title" className={styles.title}>
        {title}
      </h2>

      <div className={styles.headerActions}>
        {/* Clear button (FR-013) */}
        {showClear && onClear && (
          <button
            type="button"
            className={styles.clearButton}
            onClick={onClear}
            aria-label="Clear conversation"
            title="Clear conversation"
          >
            <svg
              width="18"
              height="18"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              aria-hidden="true"
            >
              <path d="M3 6h18" />
              <path d="M19 6v14c0 1-1 2-2 2H7c-1 0-2-1-2-2V6" />
              <path d="M8 6V4c0-1 1-2 2-2h4c1 0 2 1 2 2v2" />
              <line x1="10" y1="11" x2="10" y2="17" />
              <line x1="14" y1="11" x2="14" y2="17" />
            </svg>
          </button>
        )}

        {/* Close button (FR-010) */}
        <button
          type="button"
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chatbot"
          title="Close (Esc)"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            aria-hidden="true"
          >
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        </button>
      </div>
    </header>
  );
});

export default ChatHeader;
