/**
 * useTheme Hook
 *
 * Custom React hook for detecting Docusaurus color mode (light/dark).
 * Listens for theme changes and provides reactive state.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-011, SC-006
 * Task: T051
 */

import { useState, useEffect, useCallback } from 'react';

/**
 * Theme/color mode options
 */
export type ColorMode = 'light' | 'dark';

/**
 * Theme state returned by the hook
 */
export interface ThemeState {
  /** Current color mode */
  colorMode: ColorMode;
  /** True if dark mode is active */
  isDark: boolean;
  /** True if light mode is active */
  isLight: boolean;
}

/**
 * Detect current theme from Docusaurus data-theme attribute
 * Falls back to system preference
 */
function detectColorMode(): ColorMode {
  // SSR-safe check
  if (typeof document === 'undefined') {
    return 'light';
  }

  // Check Docusaurus data-theme attribute on <html>
  const htmlTheme = document.documentElement.getAttribute('data-theme');
  if (htmlTheme === 'dark' || htmlTheme === 'light') {
    return htmlTheme;
  }

  // Fallback to system preference
  if (typeof window !== 'undefined' && window.matchMedia) {
    const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    return prefersDark ? 'dark' : 'light';
  }

  return 'light';
}

/**
 * Get initial theme state
 */
function getInitialState(): ThemeState {
  const colorMode = detectColorMode();
  return {
    colorMode,
    isDark: colorMode === 'dark',
    isLight: colorMode === 'light',
  };
}

/**
 * useTheme - Docusaurus color mode detection hook
 *
 * Provides:
 * - Current color mode (light/dark)
 * - Boolean flags for each mode
 * - Automatic updates when theme changes
 *
 * Detects theme via:
 * 1. Docusaurus data-theme attribute on <html>
 * 2. System preference (prefers-color-scheme)
 *
 * Theme transitions complete within 200ms (SC-006).
 *
 * Usage:
 * ```tsx
 * const { isDark, colorMode } = useTheme();
 *
 * return (
 *   <div className={isDark ? styles.dark : styles.light}>
 *     Current theme: {colorMode}
 *   </div>
 * );
 * ```
 */
export function useTheme(): ThemeState {
  const [state, setState] = useState<ThemeState>(getInitialState);

  // Update state when theme changes
  const updateTheme = useCallback(() => {
    const colorMode = detectColorMode();
    setState({
      colorMode,
      isDark: colorMode === 'dark',
      isLight: colorMode === 'light',
    });
  }, []);

  useEffect(() => {
    // Skip on server
    if (typeof document === 'undefined' || typeof window === 'undefined') {
      return;
    }

    // Initial detection
    updateTheme();

    // Watch for data-theme attribute changes on <html>
    const observer = new MutationObserver((mutations) => {
      for (const mutation of mutations) {
        if (
          mutation.type === 'attributes' &&
          mutation.attributeName === 'data-theme'
        ) {
          updateTheme();
          break;
        }
      }
    });

    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme'],
    });

    // Also listen for system preference changes
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');

    const handleMediaChange = () => {
      // Only use system preference if no explicit theme is set
      const htmlTheme = document.documentElement.getAttribute('data-theme');
      if (!htmlTheme) {
        updateTheme();
      }
    };

    // Modern browsers
    if (mediaQuery.addEventListener) {
      mediaQuery.addEventListener('change', handleMediaChange);
    } else {
      // Older browsers
      mediaQuery.addListener(handleMediaChange);
    }

    return () => {
      observer.disconnect();
      if (mediaQuery.removeEventListener) {
        mediaQuery.removeEventListener('change', handleMediaChange);
      } else {
        mediaQuery.removeListener(handleMediaChange);
      }
    };
  }, [updateTheme]);

  return state;
}

export default useTheme;
