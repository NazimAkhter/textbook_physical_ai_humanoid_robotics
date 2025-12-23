/**
 * ChatToggleButton Component
 *
 * Floating action button to open/close the chat panel.
 * Fixed position in bottom-right corner.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-001, FR-016, FR-018
 */

import React from 'react';
import styles from './styles/ChatToggleButton.module.css';

export interface ChatToggleButtonProps {
  /** Whether the chat panel is currently open */
  isOpen: boolean;
  /** Callback when button is clicked */
  onClick: () => void;
  /** Accessible label for the button */
  ariaLabel?: string;
}

/**
 * ChatToggleButton - Floating action button for chat panel
 *
 * Accessibility (FR-016, FR-018):
 * - Keyboard accessible via Tab navigation
 * - Activates on Enter and Space keys
 * - ARIA label describes current state
 *
 * Usage:
 * ```tsx
 * <ChatToggleButton
 *   isOpen={isPanelOpen}
 *   onClick={togglePanel}
 * />
 * ```
 */
export function ChatToggleButton({
  isOpen,
  onClick,
  ariaLabel,
}: ChatToggleButtonProps): React.ReactElement {
  const defaultLabel = isOpen ? 'Close chatbot' : 'Open chatbot';

  return (
    <button
      type="button"
      className={`${styles.toggleButton} ${isOpen ? styles.open : ''}`}
      onClick={onClick}
      aria-label={ariaLabel || defaultLabel}
      aria-expanded={isOpen}
      aria-haspopup="dialog"
    >
      <span className={styles.icon} aria-hidden="true">
        {isOpen ? (
          // Close icon (X)
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        ) : (
          // Chat icon
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        )}
      </span>
      <span className={styles.srOnly}>
        {defaultLabel}
      </span>
    </button>
  );
}

export default ChatToggleButton;
