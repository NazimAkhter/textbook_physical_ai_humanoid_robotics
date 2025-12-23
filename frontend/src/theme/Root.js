/**
 * Root Theme Wrapper
 *
 * This file wraps the Docusaurus site with the ChatBot component.
 * Created as a custom theme wrapper per Docusaurus documentation.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-001, FR-017, T026, T027
 */

import React, { lazy, Suspense } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Lazy load ChatBot for performance (T027)
const ChatBot = lazy(() => import('@site/src/components/ChatBot/ChatBot'));
const ErrorBoundary = lazy(() => import('@site/src/components/ChatBot/ErrorBoundary'));

/**
 * ChatBot wrapper with error boundary and lazy loading
 */
function ChatBotWrapper() {
  return (
    <Suspense fallback={null}>
      <ErrorBoundary
        fallback={
          <div style={{ display: 'none' }}>
            {/* Silent fallback - ChatBot error should not affect main content */}
          </div>
        }
      >
        <ChatBot />
      </ErrorBoundary>
    </Suspense>
  );
}

/**
 * Root component that wraps the entire Docusaurus site
 *
 * Features:
 * - Renders ChatBot alongside site content
 * - Uses BrowserOnly to prevent SSR issues
 * - Lazy loads ChatBot for better initial page load
 * - ErrorBoundary prevents ChatBot errors from crashing the site
 */
export default function Root({ children }) {
  return (
    <>
      {children}
      {/* ChatBot - Browser only to avoid SSR hydration issues */}
      <BrowserOnly fallback={null}>
        {() => <ChatBotWrapper />}
      </BrowserOnly>
    </>
  );
}
