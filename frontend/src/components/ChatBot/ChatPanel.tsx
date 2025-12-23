/**
 * ChatPanel Component
 *
 * Modal container for the chat interface.
 * Responsive layout: desktop (right panel), tablet (smaller panel), mobile (bottom sheet).
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-002, FR-017, FR-018, FR-024, FR-031, FR-032
 * Phase 5 Enhancements: T047, T053 (responsive + theme hooks)
 */

import React, { useEffect, useRef, useCallback } from 'react';
import styles from './styles/ChatPanel.module.css';
import type { Message } from '../../types/message';
import type { StorageType } from '../../types/chatState';
import { ChatHeader } from './ChatHeader';
import { MessageList } from './MessageList';
import { MessageInput } from './MessageInput';
import { WelcomeMessage } from './WelcomeMessage';
import { useResponsive } from '../../hooks/useResponsive';
import { useTheme } from '../../hooks/useTheme';

export interface ChatPanelProps {
  /** Whether the panel is open */
  isOpen: boolean;
  /** Callback to close the panel */
  onClose: () => void;
  /** List of messages to display */
  messages: Message[];
  /** Whether a response is being loaded */
  isLoading: boolean;
  /** Current input value */
  inputValue: string;
  /** Callback when input changes */
  onInputChange: (value: string) => void;
  /** Callback when message is submitted */
  onSendMessage: (content: string) => void;
  /** Callback to clear conversation */
  onClearConversation?: () => void;
  /** Current storage type (for fallback notice) */
  storageType?: StorageType;
  /** Optional ref for input element (external focus management) */
  inputRef?: React.RefObject<HTMLTextAreaElement>;
  /** Optional custom welcome title */
  welcomeTitle?: string;
  /** Optional custom welcome message */
  welcomeMessage?: string;
}

/**
 * ChatPanel - Main chat interface container
 *
 * Features:
 * - Focus trap when open (FR-017)
 * - ESC key to close (FR-018)
 * - Responsive layout (FR-024, FR-031, FR-032)
 * - Theme support (FR-011)
 * - Storage fallback notice (FR-027)
 *
 * Responsive Breakpoints:
 * - Desktop (>=996px): 400px right-side panel, 80% max-height (FR-031)
 * - Tablet (768-995px): 350px right-side panel, 85% max-height (FR-032)
 * - Mobile (<768px): Bottom sheet, 70% screen height (FR-024)
 *
 * Usage:
 * ```tsx
 * <ChatPanel
 *   isOpen={isPanelOpen}
 *   onClose={closePanel}
 *   messages={messages}
 *   isLoading={isLoading}
 *   inputValue={inputValue}
 *   onInputChange={setInputValue}
 *   onSendMessage={sendMessage}
 * />
 * ```
 */
export function ChatPanel({
  isOpen,
  onClose,
  messages,
  isLoading,
  inputValue,
  onInputChange,
  onSendMessage,
  onClearConversation,
  storageType = 'session',
  inputRef: externalInputRef,
  welcomeTitle,
  welcomeMessage,
}: ChatPanelProps): React.ReactElement | null {
  const panelRef = useRef<HTMLDivElement>(null);
  const inputRef = externalInputRef || useRef<HTMLTextAreaElement>(null);

  // Responsive layout detection (T047)
  const { isMobile, deviceType } = useResponsive();

  // Theme detection (T053)
  const { isDark, colorMode } = useTheme();

  // Handle ESC key to close panel (FR-018)
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isOpen) {
        onClose();
      }
    },
    [isOpen, onClose]
  );

  // Add/remove ESC key listener
  useEffect(() => {
    if (isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      // Focus input when panel opens
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen, handleKeyDown]);

  // Prevent body scroll when panel is open on mobile
  useEffect(() => {
    if (isOpen && isMobile) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen, isMobile]);

  // Note: Removed redundant isOpen check (T092) - parent already conditionally renders
  const showWelcome = messages.length === 0 && !isLoading;

  // Build panel class names for responsive and theme support
  const panelClassNames = [
    styles.panel,
    isMobile ? styles.mobilePanel : '',
    isDark ? styles.darkPanel : '',
  ]
    .filter(Boolean)
    .join(' ');

  return (
    <>
      {/* Backdrop */}
      <div
        className={styles.backdrop}
        onClick={onClose}
        aria-hidden="true"
        data-device={deviceType}
        data-theme={colorMode}
      />

      {/* Panel */}
      <div
        ref={panelRef}
        className={panelClassNames}
        role="dialog"
        aria-modal="true"
        aria-labelledby="chatbot-title"
        data-device={deviceType}
        data-theme={colorMode}
      >
        <ChatHeader
          onClose={onClose}
          onClear={onClearConversation}
          showClear={messages.length > 0}
        />

        {/* Storage fallback notice (FR-027) */}
        {storageType === 'memory' && (
          <div className={styles.storageFallbackNotice}>
            <span className={styles.noticeIcon} aria-hidden="true">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <circle cx="12" cy="12" r="10" />
                <line x1="12" y1="16" x2="12" y2="12" />
                <line x1="12" y1="8" x2="12.01" y2="8" />
              </svg>
            </span>
            <span>Conversation history won't persist across pages</span>
          </div>
        )}

        <div className={styles.content}>
          {showWelcome ? (
            <WelcomeMessage />
          ) : (
            <MessageList messages={messages} isLoading={isLoading} />
          )}
        </div>

        <MessageInput
          ref={inputRef}
          value={inputValue}
          onChange={onInputChange}
          onSubmit={onSendMessage}
          disabled={isLoading}
          placeholder="Ask about Physical AI & Robotics..."
        />
      </div>
    </>
  );
}

export default ChatPanel;
