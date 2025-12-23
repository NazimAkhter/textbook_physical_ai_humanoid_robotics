/**
 * WelcomeMessage Component
 *
 * Empty state display when no messages exist.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-033
 * Phase 7: T074 - React.memo optimization
 */

import React, { memo } from 'react';
import styles from './styles/ChatPanel.module.css';

export interface WelcomeMessageProps {
  /** Custom welcome message (defaults to FR-033 content) */
  message?: string;
}

/**
 * Default welcome message content (FR-033)
 */
const DEFAULT_MESSAGE = "Ask me anything about this book! Try: 'What is VLA?' or 'Explain SLAM'";

/**
 * Suggested questions for users
 */
const SUGGESTIONS = [
  { label: 'What is VLA?', query: 'What is VLA?' },
  { label: 'Explain SLAM', query: 'Explain SLAM' },
  { label: 'Tell me about ROS 2', query: 'Tell me about ROS 2' },
  { label: 'What is a digital twin?', query: 'What is a digital twin?' },
];

/**
 * WelcomeMessage - Empty state with helpful suggestions
 *
 * Displays when there are no messages in the conversation.
 * Provides suggested questions to help users get started.
 *
 * Usage:
 * ```tsx
 * {messages.length === 0 && <WelcomeMessage />}
 * ```
 */
export const WelcomeMessage = memo(function WelcomeMessage({
  message = DEFAULT_MESSAGE,
}: WelcomeMessageProps): React.ReactElement {
  return (
    <div className={styles.welcomeContainer}>
      <div className={styles.welcomeIcon} aria-hidden="true">
        <svg
          width="48"
          height="48"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="1.5"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <circle cx="12" cy="12" r="10" />
          <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3" />
          <line x1="12" y1="17" x2="12.01" y2="17" />
        </svg>
      </div>

      <h3 className={styles.welcomeTitle}>
        Welcome to Book Assistant
      </h3>

      <p className={styles.welcomeMessage}>
        {message}
      </p>

      <div className={styles.suggestions}>
        <p className={styles.suggestionsLabel}>Try asking:</p>
        <ul className={styles.suggestionsList}>
          {SUGGESTIONS.map((suggestion) => (
            <li key={suggestion.label} className={styles.suggestionItem}>
              <span className={styles.suggestionBullet}>•</span>
              {suggestion.label}
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
});

export default WelcomeMessage;
