# Data Model: UI Theme & Navigation Update

**Feature**: 006-ui-theme-update
**Date**: 2025-12-16
**Phase**: 1 (Design & Contracts)

## Overview

This feature manages visual theme configuration and component styling for the Docusaurus site. The "data" in this context refers to CSS custom properties, component style configurations, and navigation structure definitions rather than traditional database entities.

---

## Entity 1: Theme Configuration

**Description**: Collection of CSS custom properties defining the global color palette for light and dark modes.

**Location**: `frontend/src/css/custom.css`

**Attributes**:

| Attribute | Type | Description | Validation Rules |
|-----------|------|-------------|------------------|
| `--ifm-color-primary` | CSS Color | Primary brand color | Must be valid hex/rgb color |
| `--ifm-color-primary-dark` | CSS Color | Darker shade of primary | Derived from primary, ~20% darker |
| `--ifm-color-primary-darker` | CSS Color | Even darker shade | Derived from primary, ~30% darker |
| `--ifm-color-primary-darkest` | CSS Color | Darkest shade | Derived from primary, ~40% darker |
| `--ifm-color-primary-light` | CSS Color | Lighter shade of primary | Derived from primary, ~20% lighter |
| `--ifm-color-primary-lighter` | CSS Color | Even lighter shade | Derived from primary, ~30% lighter |
| `--ifm-color-primary-lightest` | CSS Color | Lightest shade | Derived from primary, ~40% lighter |
| `--ifm-link-color` | CSS Color | Hyperlink color | Must meet WCAG AA contrast (4.5:1) |
| `--ifm-link-hover-color` | CSS Color | Hyperlink hover state | Must be visually distinct from link color |
| `--ifm-footer-background-color` | CSS Color | Footer background | Must contrast with footer text |
| `--ifm-footer-link-color` | CSS Color | Footer link color | Must meet WCAG AA contrast on footer bg |
| `--ifm-footer-link-hover-color` | CSS Color | Footer link hover | Must be visually distinct |

**Mode Variants**:
- **Light Mode** (`:root` selector): Teal-based palette (#115e59 primary)
- **Dark Mode** (`[data-theme='dark']` selector): Cyan-based palette (#99f6e4 primary)

**Relationships**:
- Used by all Docusaurus components globally
- Inherited by `Hero Component` via `var(--ifm-color-primary)`
- Referenced in `Footer Component` for styling

**State Transitions**:
- Mode Switch: User toggles dark mode → Docusaurus applies `[data-theme='dark']` attribute → CSS variables update → Components re-render

**Validation**:
- All colors must pass WCAG AA contrast requirements (4.5:1 for text, 3:1 for UI elements)
- Light mode primary must be #115e59 (FR-003)
- Dark mode primary must be #99f6e4 (FR-004)

---

## Entity 2: Hero Component

**Description**: Homepage hero section displaying site title, robot image, and call-to-action buttons with mode-specific background colors.

**Location**: `frontend/src/components/HomepageFeatures/index.js` + `styles.module.css`

**Attributes**:

| Attribute | Type | Description | Validation Rules |
|-----------|------|-------------|------------------|
| `title` | String | Hero section heading | Must be "Physical AI & Humanoid Robotics Book" (FR-007) |
| `backgroundColorLight` | CSS Color | Light mode background | Must be #115e59 (FR-008) |
| `backgroundColorDark` | CSS Color | Dark mode background | Must be #99f6e4 (FR-009) |
| `heroImage` | String (URL) | Robot image path | Must be `/img/hero-robot.png` (FR-010) |
| `textColorLight` | CSS Color | Light mode text | Must be white (#FFFFFF) for contrast |
| `textColorDark` | CSS Color | Dark mode text | Must be dark (#1b1b1d) for contrast |
| `buttons` | Array<Button> | CTA buttons | At least one button with link to /docs/intro |

**Relationships**:
- Rendered within `HomepageFeatures` component
- Imports `Theme Configuration` via CSS custom properties for button colors
- Links to documentation via `to` prop (React Router)

**State Transitions**:
- Component Mount: Reads current theme mode → Applies appropriate background/text colors
- Mode Toggle: Theme changes → CSS classes update → Background/text colors switch

**Validation**:
- Title must be non-empty string matching FR-007
- Background colors must exactly match FR-008 and FR-009
- Image must load without 404 error
- Text contrast must meet WCAG AA (5.2:1 light mode, 11.8:1 dark mode)
- Responsive layout must work from 320px to 2560px viewport width (FR-012)

---

## Entity 3: Navigation Component

**Description**: Top navbar configuration defining site navigation structure, logo, and module links.

**Location**: `frontend/docusaurus.config.js` → `themeConfig.navbar`

**Attributes**:

| Attribute | Type | Description | Validation Rules |
|-----------|------|-------------|------------------|
| `logo.src` | String (URL) | Navbar logo image | Must be valid image URL (FR-013) |
| `logo.alt` | String | Logo alt text | Must be descriptive for accessibility |
| `logo.href` | String | Logo click destination | Typically '/' for homepage |
| `items` | Array<NavItem> | Navigation menu items | Must include Modules dropdown (FR-014) |
| `items[].type` | String | Nav item type | 'dropdown', 'doc', 'docSidebar', 'html' |
| `items[].label` | String | Display text | Must be clear and concise |
| `items[].items` | Array<NavItem> | Dropdown children | For 'dropdown' type only |

**Modules Dropdown Structure** (FR-014):

```js
{
  type: 'dropdown',
  label: 'Modules',
  items: [
    {label: 'Module 1: ROS 2 Nervous System', to: '/docs/category/module-1-ros-2-nervous-system'},
    {label: 'Module 2: Digital Twins', to: '/docs/category/module-2-digital-twins'},
    {label: 'Module 3: AI-Robot Brain (Isaac)', to: '/docs/category/module-3-ai-robot-brain'},
    {label: 'Module 4: VLA Systems', to: '/docs/category/module-4-vla-systems'},
  ],
}
```

**Relationships**:
- References `Theme Configuration` for navbar colors
- Links to module category pages (auto-generated by Docusaurus)
- Logo image sourced from `static/img/` or external URL

**Validation**:
- Logo source must be https://cdn3d.iconscout.com/3d/premium/thumb/robot-3d-icon-png-download-9911391.png (FR-013)
- Must contain exactly 4 module links (Module 1-4) in dropdown (FR-014)
- All links must resolve to valid routes (no 404s)
- Logo must display at reasonable size (~40px height)

---

## Entity 4: Footer Component

**Description**: Site footer displaying copyright, links, and social media with theme-aware styling.

**Location**: `frontend/docusaurus.config.js` → `themeConfig.footer` + CSS variables

**Attributes**:

| Attribute | Type | Description | Validation Rules |
|-----------|------|-------------|------------------|
| `style` | String | Footer preset style | 'light' or 'dark' (auto-adapts with theme) |
| `links` | Array<LinkGroup> | Footer link groups | Organized by category |
| `copyright` | String | Copyright notice | Must include year and attribution |
| `backgroundColor` | CSS Color | Background color | From `--ifm-footer-background-color` |
| `linkColor` | CSS Color | Link text color | From `--ifm-footer-link-color` (FR-015, FR-016) |
| `linkHoverColor` | CSS Color | Link hover color | From `--ifm-footer-link-hover-color` |

**Relationships**:
- Inherits colors from `Theme Configuration` CSS variables
- Link groups can reference external URLs or internal routes

**State Transitions**:
- Theme Toggle: Mode changes → Footer background/link colors update via CSS variables

**Validation**:
- Background color must align with theme (FR-015): Light gray (#f8f9fa) in light mode, dark gray (#1b1b1d) in dark mode
- Link colors must use theme palette (FR-016): Teal in light mode, cyan in dark mode
- All links must be valid URLs or routes
- Copyright text must be present and up-to-date

---

## Entity 5: Color Mode

**Description**: Represents the current theme mode (light or dark) that determines which color palette variant to apply.

**Location**: Docusaurus internal state + `[data-theme]` HTML attribute

**Attributes**:

| Attribute | Type | Description | Validation Rules |
|-----------|------|-------------|------------------|
| `mode` | Enum | Current theme mode | Must be 'light' or 'dark' |
| `userPreference` | Enum | User's stored preference | 'light', 'dark', or 'system' (follows OS) |
| `systemPreference` | Enum | OS-level theme | Detected from `prefers-color-scheme` media query |

**Relationships**:
- Affects all entities above by switching CSS variable sets
- Persisted in localStorage as `theme` key
- Triggers re-render of components on mode change

**State Transitions**:

```
Initial Load → Check localStorage ('theme') →
  If set: Apply stored preference
  If not set: Apply system preference (prefers-color-scheme)

User Clicks Toggle →
  Update localStorage →
  Update [data-theme] attribute →
  CSS variables switch →
  Components re-render

OS Theme Changes (if userPreference='system') →
  Detect new system preference →
  Update [data-theme] attribute →
  CSS variables switch
```

**Validation**:
- Mode must be one of 'light' or 'dark' (no invalid states)
- Toggle must respond within 200ms (FR-003 success criterion SC-003)

---

## Entity Relationships Diagram

```
┌────────────────────────┐
│  Theme Configuration   │ (CSS Variables)
│  ──────────────────   │
│  Primary Colors        │
│  Link Colors           │
│  Footer Colors         │
└───────────┬────────────┘
            │ inherits
            │
    ┌───────┴───────┬──────────────┬────────────────┐
    │               │              │                │
    ▼               ▼              ▼                ▼
┌─────────┐   ┌──────────┐   ┌─────────────┐  ┌─────────────┐
│  Hero   │   │ Navigation│   │   Footer    │  │  All Other  │
│Component│   │ Component │   │  Component  │  │ Components  │
└─────────┘   └──────────┘   └─────────────┘  └─────────────┘
    │
    │ styled by
    ▼
┌──────────────┐
│  Color Mode  │ (Light/Dark State)
│  ───────────│
│  Triggers    │──────► Updates [data-theme] attribute
│  CSS switch  │       → CSS variables change
└──────────────┘       → Components re-render
```

---

## Validation Rules Summary

**Global Rules**:
1. All colors must meet WCAG AA contrast requirements
2. No implementation must break existing content or documentation
3. Responsive design must work 320px-2560px viewports
4. Build must complete without errors

**Entity-Specific Rules**:
- **Theme Configuration**: Primary colors must exactly match spec (FR-003, FR-004)
- **Hero Component**: Title, backgrounds, image must match spec (FR-007 to FR-010)
- **Navigation**: Logo URL and 4 module links required (FR-013, FR-014)
- **Footer**: Colors must align with global theme (FR-015, FR-016)
- **Color Mode**: Toggle must respond <200ms (SC-003)

**Cross-Entity Rules**:
- All components must respect Color Mode state
- All components must inherit from Theme Configuration where applicable
- Changes to Theme Configuration propagate to all child components
