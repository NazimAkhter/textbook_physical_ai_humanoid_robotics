/**
 * Markdown Utility
 *
 * Provides react-markdown configuration with syntax highlighting.
 * Uses rehype-prism-plus for code block highlighting.
 *
 * Feature: 007-rag-chatbot-ui
 * Based on: research.md (Task 1)
 * Requirements: FR-006, FR-007
 */

import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import rehypePrism from 'rehype-prism-plus';
import type { Components } from 'react-markdown';

/**
 * Props for MarkdownRenderer component
 */
export interface MarkdownRendererProps {
  /** Markdown content to render */
  content: string;
  /** Additional CSS class names */
  className?: string;
}

/**
 * Custom components for react-markdown
 * Adds styling classes compatible with Docusaurus/Prism themes
 */
const markdownComponents: Components = {
  // Code blocks with language detection
  code({ className, children, ...props }) {
    const match = /language-(\w+)/.exec(className || '');
    const language = match ? match[1] : '';

    // Inline code (no language specified, single line)
    if (!language && typeof children === 'string' && !children.includes('\n')) {
      return (
        <code className="chatbot-inline-code" {...props}>
          {children}
        </code>
      );
    }

    // Block code with syntax highlighting
    return (
      <code className={className} {...props}>
        {children}
      </code>
    );
  },

  // Pre wrapper for code blocks
  pre({ children, ...props }) {
    return (
      <pre className="chatbot-code-block" {...props}>
        {children}
      </pre>
    );
  },

  // Links open in new tab for external URLs
  a({ href, children, ...props }) {
    const isExternal = href?.startsWith('http');
    return (
      <a
        href={href}
        target={isExternal ? '_blank' : undefined}
        rel={isExternal ? 'noopener noreferrer' : undefined}
        className="chatbot-link"
        {...props}
      >
        {children}
      </a>
    );
  },

  // Styled lists
  ul({ children, ...props }) {
    return (
      <ul className="chatbot-list chatbot-list-ul" {...props}>
        {children}
      </ul>
    );
  },

  ol({ children, ...props }) {
    return (
      <ol className="chatbot-list chatbot-list-ol" {...props}>
        {children}
      </ol>
    );
  },

  // Styled blockquotes
  blockquote({ children, ...props }) {
    return (
      <blockquote className="chatbot-blockquote" {...props}>
        {children}
      </blockquote>
    );
  },

  // Styled headings (scale down for chat context)
  h1({ children, ...props }) {
    return (
      <h4 className="chatbot-heading chatbot-h1" {...props}>
        {children}
      </h4>
    );
  },
  h2({ children, ...props }) {
    return (
      <h5 className="chatbot-heading chatbot-h2" {...props}>
        {children}
      </h5>
    );
  },
  h3({ children, ...props }) {
    return (
      <h6 className="chatbot-heading chatbot-h3" {...props}>
        {children}
      </h6>
    );
  },

  // Styled paragraphs
  p({ children, ...props }) {
    return (
      <p className="chatbot-paragraph" {...props}>
        {children}
      </p>
    );
  },

  // Styled strong/bold
  strong({ children, ...props }) {
    return (
      <strong className="chatbot-strong" {...props}>
        {children}
      </strong>
    );
  },

  // Styled emphasis/italic
  em({ children, ...props }) {
    return (
      <em className="chatbot-em" {...props}>
        {children}
      </em>
    );
  },
};

/**
 * Markdown Renderer Component
 *
 * Renders markdown content with syntax highlighting and custom styling.
 *
 * Usage:
 * ```tsx
 * import { MarkdownRenderer } from '../utils/markdown';
 *
 * <MarkdownRenderer content={message.content} />
 * ```
 */
export function MarkdownRenderer({
  content,
  className = '',
}: MarkdownRendererProps): React.ReactElement {
  return (
    <div className={`chatbot-markdown ${className}`.trim()}>
      <ReactMarkdown
        remarkPlugins={[remarkGfm]}
        rehypePlugins={[rehypePrism]}
        components={markdownComponents}
      >
        {content}
      </ReactMarkdown>
    </div>
  );
}

/**
 * Render markdown to React elements (function form)
 *
 * Usage:
 * ```tsx
 * import { renderMarkdown } from '../utils/markdown';
 *
 * const element = renderMarkdown('**Hello** world');
 * ```
 */
export function renderMarkdown(content: string): React.ReactElement {
  return <MarkdownRenderer content={content} />;
}

export default MarkdownRenderer;
