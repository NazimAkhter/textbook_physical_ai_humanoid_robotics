/**
 * LoadingIndicator Component
 *
 * Animated loading indicator for pending assistant responses.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-004
 * Phase 7: T074 - React.memo optimization
 */

import React, { memo } from 'react';
import styles from './styles/Message.module.css';

export interface LoadingIndicatorProps {
  /** Additional CSS class names */
  className?: string;
}

/**
 * LoadingIndicator - Animated dots for loading state
 *
 * Displays three animated dots to indicate the assistant is typing.
 * Uses CSS animations for smooth, performant animation.
 *
 * Usage:
 * ```tsx
 * {isLoading && <LoadingIndicator />}
 * ```
 */
export const LoadingIndicator = memo(function LoadingIndicator({
  className = '',
}: LoadingIndicatorProps): React.ReactElement {
  return (
    <div
      className={`${styles.loadingContainer} ${className}`.trim()}
      role="status"
      aria-label="Loading response"
    >
      <div className={styles.loadingDots}>
        <span className={styles.dot} style={{ animationDelay: '0ms' }} />
        <span className={styles.dot} style={{ animationDelay: '150ms' }} />
        <span className={styles.dot} style={{ animationDelay: '300ms' }} />
      </div>
      <span className={styles.srOnly}>
        Thinking...
      </span>
    </div>
  );
});

export default LoadingIndicator;
