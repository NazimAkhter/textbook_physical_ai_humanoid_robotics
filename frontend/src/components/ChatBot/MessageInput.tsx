/**
 * MessageInput Component
 *
 * Text input with submit button for sending messages.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-003, FR-015
 */

import React, { forwardRef, useCallback, KeyboardEvent, ChangeEvent } from 'react';
import styles from './styles/ChatPanel.module.css';

export interface MessageInputProps {
  /** Current input value */
  value: string;
  /** Callback when input changes */
  onChange: (value: string) => void;
  /** Callback when message is submitted */
  onSubmit: (content: string) => void;
  /** Whether the input is disabled */
  disabled?: boolean;
  /** Placeholder text */
  placeholder?: string;
}

/**
 * MessageInput - Text input with submit functionality
 *
 * Features:
 * - Enter to submit (FR-003)
 * - Shift+Enter for newline
 * - Empty message validation (FR-015)
 * - Disabled state during loading
 *
 * Usage:
 * ```tsx
 * <MessageInput
 *   value={inputValue}
 *   onChange={setInputValue}
 *   onSubmit={sendMessage}
 *   disabled={isLoading}
 * />
 * ```
 */
export const MessageInput = forwardRef<HTMLTextAreaElement, MessageInputProps>(
  function MessageInput(
    {
      value,
      onChange,
      onSubmit,
      disabled = false,
      placeholder = 'Type your question...',
    },
    ref
  ): React.ReactElement {
    // Handle input change
    const handleChange = useCallback(
      (e: ChangeEvent<HTMLTextAreaElement>) => {
        onChange(e.target.value);
      },
      [onChange]
    );

    // Handle key press (Enter to submit, Shift+Enter for newline)
    const handleKeyDown = useCallback(
      (e: KeyboardEvent<HTMLTextAreaElement>) => {
        if (e.key === 'Enter' && !e.shiftKey) {
          e.preventDefault();
          const trimmedValue = value.trim();
          if (trimmedValue && !disabled) {
            onSubmit(trimmedValue);
          }
        }
      },
      [value, disabled, onSubmit]
    );

    // Handle submit button click
    const handleSubmit = useCallback(() => {
      const trimmedValue = value.trim();
      if (trimmedValue && !disabled) {
        onSubmit(trimmedValue);
      }
    }, [value, disabled, onSubmit]);

    const isSubmitDisabled = disabled || !value.trim();

    return (
      <div className={styles.inputContainer}>
        <textarea
          ref={ref}
          className={styles.input}
          value={value}
          onChange={handleChange}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          disabled={disabled}
          rows={1}
          aria-label="Message input"
          aria-describedby="input-hint"
        />

        <button
          type="button"
          className={styles.submitButton}
          onClick={handleSubmit}
          disabled={isSubmitDisabled}
          aria-label="Send message"
          title="Send (Enter)"
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
            <line x1="22" y1="2" x2="11" y2="13" />
            <polygon points="22 2 15 22 11 13 2 9 22 2" />
          </svg>
        </button>

        <span id="input-hint" className={styles.srOnly}>
          Press Enter to send, Shift+Enter for new line
        </span>
      </div>
    );
  }
);

export default MessageInput;
