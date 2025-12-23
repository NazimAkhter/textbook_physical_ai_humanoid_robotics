/**
 * MessageList Component
 *
 * Scrollable container for displaying chat messages.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-008, FR-009
 * Phase 4 Enhancements: T035, T036 (auto-scroll, scroll stability)
 */

import React, { useRef, useEffect, useCallback, useState } from 'react';
import styles from './styles/ChatPanel.module.css';
import messageListStyles from './styles/MessageList.module.css';
import { Message } from './Message';
import { LoadingIndicator } from './LoadingIndicator';
import type { Message as MessageType } from '../../types/message';

export interface MessageListProps {
  /** List of messages to display */
  messages: MessageType[];
  /** Whether a response is being loaded */
  isLoading: boolean;
  /** Additional CSS class names */
  className?: string;
}

/**
 * Threshold for considering user "at bottom" of scroll (in pixels)
 * Allows for some tolerance when auto-scrolling
 */
const SCROLL_THRESHOLD = 100;

/**
 * MessageList - Scrollable message container
 *
 * Features:
 * - Auto-scroll to latest message (FR-009, T035)
 * - Scroll position stability when user scrolls up (T036)
 * - Loading indicator for pending responses (FR-004)
 * - Smooth scrolling
 *
 * Usage:
 * ```tsx
 * <MessageList messages={messages} isLoading={isLoading} />
 * ```
 */
export function MessageList({
  messages,
  isLoading,
  className = '',
}: MessageListProps): React.ReactElement {
  const listRef = useRef<HTMLDivElement>(null);
  const bottomRef = useRef<HTMLDivElement>(null);

  // Track if user has scrolled up (T036: scroll position stability)
  const [isUserScrolledUp, setIsUserScrolledUp] = useState(false);

  // Track previous message count to detect new messages
  const prevMessageCount = useRef(messages.length);

  /**
   * Check if scroll position is near the bottom
   */
  const isNearBottom = useCallback((): boolean => {
    if (!listRef.current) return true;

    const { scrollTop, scrollHeight, clientHeight } = listRef.current;
    return scrollHeight - scrollTop - clientHeight < SCROLL_THRESHOLD;
  }, []);

  /**
   * Scroll to bottom programmatically
   */
  const scrollToBottom = useCallback((behavior: ScrollBehavior = 'smooth') => {
    if (bottomRef.current) {
      bottomRef.current.scrollIntoView({ behavior });
    }
  }, []);

  /**
   * Handle scroll events to track user scroll position (T036)
   */
  const handleScroll = useCallback(() => {
    const nearBottom = isNearBottom();
    setIsUserScrolledUp(!nearBottom);
  }, [isNearBottom]);

  /**
   * Auto-scroll to bottom when new messages arrive (T035, FR-009)
   * Only auto-scroll if user is already near bottom (T036: stability)
   */
  useEffect(() => {
    const newMessageCount = messages.length;
    const hasNewMessages = newMessageCount > prevMessageCount.current;

    // Update previous count
    prevMessageCount.current = newMessageCount;

    // Auto-scroll conditions:
    // 1. New messages arrived AND user was at/near bottom
    // 2. OR loading state changed to true (waiting for response)
    // 3. OR this is the initial load (messages exist, user hasn't scrolled)
    if (hasNewMessages && !isUserScrolledUp) {
      // Small delay to ensure DOM is updated
      requestAnimationFrame(() => {
        scrollToBottom();
      });
    }
  }, [messages, isUserScrolledUp, scrollToBottom]);

  /**
   * Auto-scroll when loading indicator appears (user sent message)
   */
  useEffect(() => {
    if (isLoading && !isUserScrolledUp) {
      requestAnimationFrame(() => {
        scrollToBottom();
      });
    }
  }, [isLoading, isUserScrolledUp, scrollToBottom]);

  /**
   * Reset scroll-up state when user manually scrolls back to bottom
   */
  useEffect(() => {
    const listElement = listRef.current;
    if (!listElement) return;

    listElement.addEventListener('scroll', handleScroll, { passive: true });

    return () => {
      listElement.removeEventListener('scroll', handleScroll);
    };
  }, [handleScroll]);

  // Determine if scroll indicator should show
  const showScrollIndicator = messages.length > 0 && isUserScrolledUp;

  return (
    <div className={messageListStyles.container}>
      <div
        ref={listRef}
        className={`${styles.messageList} ${messageListStyles.scrollArea} ${className}`.trim()}
        role="log"
        aria-live="polite"
        aria-label="Chat messages"
      >
        {messages.map((message) => (
          <Message key={message.id} message={message} />
        ))}

        {/* Loading indicator */}
        {isLoading && (
          <div className={styles.loadingWrapper}>
            <LoadingIndicator />
          </div>
        )}

        {/* Scroll anchor */}
        <div ref={bottomRef} aria-hidden="true" />
      </div>

      {/* Scroll to bottom indicator (T040) */}
      {showScrollIndicator && (
        <button
          type="button"
          className={messageListStyles.scrollIndicator}
          onClick={() => {
            setIsUserScrolledUp(false);
            scrollToBottom();
          }}
          aria-label="Scroll to latest message"
        >
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
            <polyline points="6 9 12 15 18 9" />
          </svg>
          <span className={messageListStyles.srOnly}>New messages below</span>
        </button>
      )}
    </div>
  );
}

export default MessageList;
