/**
 * useResponsive Hook
 *
 * Custom React hook for detecting viewport breakpoints and responsive layout.
 * Matches Docusaurus/Infima breakpoint system.
 *
 * Feature: 007-rag-chatbot-ui
 * Requirements: FR-024, FR-031, FR-032
 * Task: T046
 */

import { useState, useEffect, useCallback } from 'react';

/**
 * Viewport breakpoints matching Infima/Docusaurus
 */
export const BREAKPOINTS = {
  /** Mobile: < 768px */
  mobile: 768,
  /** Tablet: 768px - 995px */
  tablet: 996,
  /** Desktop: >= 996px */
  desktop: 996,
} as const;

/**
 * Device type based on viewport width
 */
export type DeviceType = 'mobile' | 'tablet' | 'desktop';

/**
 * Responsive state returned by the hook
 */
export interface ResponsiveState {
  /** Current device type based on viewport */
  deviceType: DeviceType;
  /** True if viewport is mobile (<768px) */
  isMobile: boolean;
  /** True if viewport is tablet (768px - 995px) */
  isTablet: boolean;
  /** True if viewport is desktop (>=996px) */
  isDesktop: boolean;
  /** Current viewport width in pixels */
  width: number;
  /** Current viewport height in pixels */
  height: number;
}

/**
 * Determine device type from viewport width
 */
function getDeviceType(width: number): DeviceType {
  if (width < BREAKPOINTS.mobile) {
    return 'mobile';
  }
  if (width < BREAKPOINTS.tablet) {
    return 'tablet';
  }
  return 'desktop';
}

/**
 * Get initial state (SSR-safe)
 */
function getInitialState(): ResponsiveState {
  // SSR: default to desktop
  if (typeof window === 'undefined') {
    return {
      deviceType: 'desktop',
      isMobile: false,
      isTablet: false,
      isDesktop: true,
      width: 1024,
      height: 768,
    };
  }

  const width = window.innerWidth;
  const height = window.innerHeight;
  const deviceType = getDeviceType(width);

  return {
    deviceType,
    isMobile: deviceType === 'mobile',
    isTablet: deviceType === 'tablet',
    isDesktop: deviceType === 'desktop',
    width,
    height,
  };
}

/**
 * useResponsive - Viewport breakpoint detection hook
 *
 * Provides:
 * - Device type detection (mobile/tablet/desktop)
 * - Boolean flags for each device type
 * - Current viewport dimensions
 * - Automatic updates on resize
 *
 * Breakpoints:
 * - Mobile: < 768px (FR-024: bottom sheet layout)
 * - Tablet: 768px - 995px (FR-032: 350px panel)
 * - Desktop: >= 996px (FR-031: 400px panel)
 *
 * Usage:
 * ```tsx
 * const { isMobile, isTablet, isDesktop, deviceType } = useResponsive();
 *
 * return (
 *   <div className={isMobile ? styles.mobilePanel : styles.desktopPanel}>
 *     ...
 *   </div>
 * );
 * ```
 */
export function useResponsive(): ResponsiveState {
  const [state, setState] = useState<ResponsiveState>(getInitialState);

  // Handle resize events
  const handleResize = useCallback(() => {
    const width = window.innerWidth;
    const height = window.innerHeight;
    const deviceType = getDeviceType(width);

    setState({
      deviceType,
      isMobile: deviceType === 'mobile',
      isTablet: deviceType === 'tablet',
      isDesktop: deviceType === 'desktop',
      width,
      height,
    });
  }, []);

  // Set up resize listener
  useEffect(() => {
    // Skip on server
    if (typeof window === 'undefined') return;

    // Initial measurement
    handleResize();

    // Listen for resize events with debouncing via passive listener
    window.addEventListener('resize', handleResize, { passive: true });

    // Also listen for orientation changes on mobile
    window.addEventListener('orientationchange', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
      window.removeEventListener('orientationchange', handleResize);
    };
  }, [handleResize]);

  return state;
}

export default useResponsive;
