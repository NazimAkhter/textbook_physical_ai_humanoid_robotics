/**
 * Error Logger Utility
 *
 * Provides console logging with optional production callback hook.
 * Ensures privacy-safe error messages (no sensitive data).
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-028, FR-029, FR-030
 */

/**
 * Error severity levels
 */
export type ErrorLevel = 'error' | 'warn' | 'info';

/**
 * Error context for logging
 */
export interface ErrorContext {
  component?: string;
  action?: string;
  userId?: never; // Never log user IDs (FR-030)
  messageContent?: never; // Never log message content (FR-030)
  [key: string]: unknown;
}

/**
 * Production error callback type
 */
export type ErrorCallback = (
  error: Error,
  level: ErrorLevel,
  context?: ErrorContext
) => void;

/**
 * Global error callback for production error tracking (FR-029)
 * Set this to integrate with Sentry, LogRocket, etc.
 */
let productionErrorCallback: ErrorCallback | null = null;

/**
 * Set the production error callback
 *
 * @param callback Function to call for production error tracking
 *
 * Usage:
 * ```typescript
 * // In Root.tsx or app initialization
 * import { setErrorCallback } from '../utils/errorLogger';
 *
 * setErrorCallback((error, level, context) => {
 *   window.Sentry?.captureException(error, { extra: context });
 * });
 * ```
 */
export function setErrorCallback(callback: ErrorCallback | null): void {
  productionErrorCallback = callback;
}

/**
 * Get the current error callback (for testing)
 */
export function getErrorCallback(): ErrorCallback | null {
  return productionErrorCallback;
}

/**
 * Log an error with optional context
 *
 * @param error Error object or message
 * @param level Severity level (error, warn, info)
 * @param context Additional context (privacy-safe)
 *
 * Usage:
 * ```typescript
 * import { logError } from '../utils/errorLogger';
 *
 * try {
 *   await riskyOperation();
 * } catch (e) {
 *   logError(e as Error, 'error', { component: 'ChatPanel', action: 'sendMessage' });
 * }
 * ```
 */
export function logError(
  error: Error | string,
  level: ErrorLevel = 'error',
  context?: ErrorContext
): void {
  const errorObj = typeof error === 'string' ? new Error(error) : error;

  // Sanitize context to ensure no sensitive data (FR-030)
  const sanitizedContext = context
    ? Object.fromEntries(
        Object.entries(context).filter(
          ([key]) => !['userId', 'messageContent', 'userInput', 'personalData'].includes(key)
        )
      )
    : undefined;

  // Console logging (development + production)
  const prefix = '[ChatBot]';
  const contextStr = sanitizedContext
    ? ` | Context: ${JSON.stringify(sanitizedContext)}`
    : '';

  switch (level) {
    case 'error':
      console.error(`${prefix} ERROR: ${errorObj.message}${contextStr}`, errorObj);
      break;
    case 'warn':
      console.warn(`${prefix} WARN: ${errorObj.message}${contextStr}`);
      break;
    case 'info':
      console.info(`${prefix} INFO: ${errorObj.message}${contextStr}`);
      break;
  }

  // Production callback (if configured)
  if (productionErrorCallback) {
    try {
      productionErrorCallback(errorObj, level, sanitizedContext);
    } catch (callbackError) {
      // Don't let callback errors break the app
      console.error('[ChatBot] Error callback failed:', callbackError);
    }
  }
}

/**
 * Convenience function for error-level logging
 */
export function logChatError(
  error: Error | string,
  context?: ErrorContext
): void {
  logError(error, 'error', context);
}

/**
 * Convenience function for warning-level logging
 */
export function logChatWarning(
  message: string,
  context?: ErrorContext
): void {
  logError(new Error(message), 'warn', context);
}

/**
 * Convenience function for info-level logging
 */
export function logChatInfo(
  message: string,
  context?: ErrorContext
): void {
  logError(new Error(message), 'info', context);
}
