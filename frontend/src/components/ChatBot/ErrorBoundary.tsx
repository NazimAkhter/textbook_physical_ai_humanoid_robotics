/**
 * ChatBot Error Boundary
 *
 * React error boundary for catching and handling chat component errors.
 * Provides fallback UI with reload option for user recovery.
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: research.md (Task 5)
 * Requirements: FR-028, FR-029, FR-030
 */

import React, { Component, ReactNode, ErrorInfo } from 'react';
import { logError } from '../../utils/errorLogger';

/**
 * Props for ErrorBoundary component
 */
export interface ErrorBoundaryProps {
  /** Child components to wrap */
  children: ReactNode;

  /**
   * Optional callback for production error tracking (FR-029)
   * Called when an error is caught, in addition to console logging
   */
  onError?: (error: Error, errorInfo: ErrorInfo) => void;

  /** Custom fallback UI (optional) */
  fallback?: ReactNode;
}

/**
 * Internal state for error boundary
 */
interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
}

/**
 * ChatBot Error Boundary Component
 *
 * Catches JavaScript errors in child component tree and displays fallback UI.
 * Provides a "Reload Chat" button for user recovery without page refresh.
 *
 * Usage:
 * ```tsx
 * import ErrorBoundary from './ErrorBoundary';
 *
 * <ErrorBoundary onError={(err, info) => sendToSentry(err)}>
 *   <ChatBot />
 * </ErrorBoundary>
 * ```
 */
class ErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
    };
  }

  /**
   * Update state when an error occurs
   * Called during render phase
   */
  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    return {
      hasError: true,
      error,
    };
  }

  /**
   * Log error and call optional callback
   * Called during commit phase
   */
  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    // Log to console with privacy-safe context (FR-030)
    logError(error, 'error', {
      component: 'ChatBot',
      action: 'errorBoundary',
      componentStack: errorInfo.componentStack?.substring(0, 200), // Truncate for brevity
    });

    // Call optional production error callback (FR-029)
    if (this.props.onError) {
      try {
        this.props.onError(error, errorInfo);
      } catch (callbackError) {
        console.error('[ErrorBoundary] onError callback failed:', callbackError);
      }
    }
  }

  /**
   * Reset error state to retry rendering
   */
  handleReload = (): void => {
    this.setState({
      hasError: false,
      error: null,
    });
  };

  render(): ReactNode {
    if (this.state.hasError) {
      // Custom fallback UI if provided
      if (this.props.fallback) {
        return this.props.fallback;
      }

      // Default fallback UI with reload button
      return (
        <div className="chatbot-error-boundary" style={styles.container}>
          <div style={styles.content}>
            <span style={styles.icon}>⚠️</span>
            <h3 style={styles.title}>Chat Error</h3>
            <p style={styles.message}>
              Something went wrong with the chat interface.
            </p>
            <button
              onClick={this.handleReload}
              style={styles.button}
              type="button"
              aria-label="Reload chatbot"
            >
              Reload Chat
            </button>

            {/* Show error details in development only (FR-030) */}
            {process.env.NODE_ENV === 'development' && this.state.error && (
              <details style={styles.details}>
                <summary style={styles.summary}>Error Details</summary>
                <pre style={styles.pre}>
                  {this.state.error.message}
                  {'\n\n'}
                  {this.state.error.stack}
                </pre>
              </details>
            )}
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

/**
 * Inline styles for fallback UI
 * Using inline styles to avoid CSS module dependency in error state
 */
const styles: Record<string, React.CSSProperties> = {
  container: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    padding: '24px',
    backgroundColor: 'var(--ifm-background-color, #fff)',
    borderRadius: '8px',
    border: '1px solid var(--ifm-color-emphasis-300, #ddd)',
    margin: '16px',
  },
  content: {
    textAlign: 'center',
    maxWidth: '300px',
  },
  icon: {
    fontSize: '48px',
    marginBottom: '16px',
    display: 'block',
  },
  title: {
    margin: '0 0 8px 0',
    fontSize: '18px',
    fontWeight: 600,
    color: 'var(--ifm-color-content, #1c1e21)',
  },
  message: {
    margin: '0 0 16px 0',
    fontSize: '14px',
    color: 'var(--ifm-color-content-secondary, #606770)',
  },
  button: {
    padding: '10px 20px',
    fontSize: '14px',
    fontWeight: 500,
    color: '#fff',
    backgroundColor: 'var(--ifm-color-primary, #3578e5)',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    transition: 'background-color 0.2s',
  },
  details: {
    marginTop: '16px',
    textAlign: 'left',
  },
  summary: {
    fontSize: '12px',
    color: 'var(--ifm-color-content-secondary, #606770)',
    cursor: 'pointer',
  },
  pre: {
    fontSize: '11px',
    backgroundColor: 'var(--ifm-code-background, #f6f8fa)',
    padding: '8px',
    borderRadius: '4px',
    overflow: 'auto',
    maxHeight: '150px',
    marginTop: '8px',
  },
};

export default ErrorBoundary;
