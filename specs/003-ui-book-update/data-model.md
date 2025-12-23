# Data Model: UI Update for Physical AI Book Website

**Feature**: `003-ui-book-update`
**Date**: 2024-12-15

## Overview

This feature primarily involves UI configuration and component data, not persistent storage. The "data model" describes the structure of configuration objects and component props used in the UI layer.

## Entities

### 1. Module Card

**Description**: Represents a learning module displayed on the homepage.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `number` | number | Yes | Module number (1-5) |
| `title` | string | Yes | Short module title (max 50 chars) |
| `description` | string | Yes | Brief description (max 120 chars) |
| `href` | string | Yes | URL path to module content |
| `status` | enum | Yes | "available" or "coming-soon" |

**Validation Rules**:
- `number` must be 1-5 (per constitution: 5 fixed modules)
- `title` truncated with ellipsis if > 50 chars
- `href` must be a valid internal path starting with `/docs/`
- `status` determines if card is clickable or shows "Coming Soon" badge

**Example**:
```javascript
{
  number: 1,
  title: "ROS 2 as the Robotic Nervous System",
  description: "Learn how ROS 2 enables communication between sensors, controllers, and actuators.",
  href: "/docs/module-01-ros2-nervous-system",
  status: "available"
}
```

### 2. Theme Configuration

**Description**: CSS custom properties defining the site-wide color palette.

| Property | Light Mode | Dark Mode | Usage |
|----------|------------|-----------|-------|
| `--ifm-color-primary` | `#6366f1` | `#818cf8` | Primary brand color |
| `--ifm-color-primary-dark` | `#4f46e5` | `#6366f1` | Darker primary shade |
| `--ifm-color-primary-darker` | `#4338ca` | `#4f46e5` | Even darker shade |
| `--ifm-color-primary-darkest` | `#3730a3` | `#4338ca` | Darkest shade |
| `--ifm-color-primary-light` | `#818cf8` | `#a5b4fc` | Lighter primary shade |
| `--ifm-color-primary-lighter` | `#a5b4fc` | `#c7d2fe` | Even lighter shade |
| `--ifm-color-primary-lightest` | `#c7d2fe` | `#e0e7ff` | Lightest shade |
| `--ifm-background-color` | `#ffffff` | `#1e1b4b` | Page background |

**Validation Rules**:
- All colors must pass WCAG AA contrast (4.5:1 for text)
- Light/dark mode colors must be complementary
- Primary color variants must form a cohesive gradient

### 3. Navigation Item

**Description**: Entry in navbar or footer navigation.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `label` | string | Yes | Display text |
| `to` | string | Conditional | Internal link path |
| `href` | string | Conditional | External link URL |
| `position` | enum | Yes (navbar) | "left" or "right" |
| `type` | string | No | Special types: "docSidebar", "dropdown" |

**Validation Rules**:
- Either `to` (internal) or `href` (external) must be provided, not both
- `position` only applies to navbar items
- Internal links must resolve to existing pages

**Example (Navbar)**:
```javascript
{
  type: 'docSidebar',
  sidebarId: 'tutorialSidebar',
  position: 'left',
  label: 'Modules'
}
```

### 4. Hero Section Configuration

**Description**: Content for homepage hero banner (via `docusaurus.config.js`).

| Field | Type | Source | Description |
|-------|------|--------|-------------|
| `title` | string | `siteConfig.title` | Main heading |
| `tagline` | string | `siteConfig.tagline` | Subtitle text |
| `ctaText` | string | Hardcoded | Button label |
| `ctaLink` | string | Hardcoded | Button destination |

**Values**:
```javascript
{
  title: "Physical AI Book",
  tagline: "Master humanoid robotics with ROS 2, digital twins, and Vision-Language-Action systems",
  ctaText: "Start Learning",
  ctaLink: "/docs/module-01-ros2-nervous-system"
}
```

### 5. Footer Configuration

**Description**: Footer content structure in `docusaurus.config.js`.

| Field | Type | Description |
|-------|------|-------------|
| `style` | enum | "dark" or "light" |
| `links` | array | Array of link sections |
| `copyright` | string | Copyright text |

**Structure**:
```javascript
{
  style: 'dark',
  links: [
    {
      title: 'Learn',
      items: [
        { label: 'Modules', to: '/docs/intro' }
      ]
    },
    {
      title: 'More',
      items: [
        { label: 'GitHub', href: 'https://github.com/[owner]/[repo]' }
      ]
    }
  ],
  copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book.`
}
```

## Relationships

```
┌─────────────────┐     ┌──────────────────┐
│   Homepage      │────▶│   Hero Section   │
│                 │     │   (config)       │
│                 │     └──────────────────┘
│                 │
│                 │     ┌──────────────────┐
│                 │────▶│   Module Cards   │
│                 │     │   (array)        │
└─────────────────┘     └──────────────────┘
        │
        │               ┌──────────────────┐
        └──────────────▶│   Theme Config   │
                        │   (CSS vars)     │
                        └──────────────────┘

┌─────────────────┐     ┌──────────────────┐
│   Site-wide     │────▶│   Navbar Items   │
│                 │     │   (config)       │
│                 │     └──────────────────┘
│                 │
│                 │     ┌──────────────────┐
│                 │────▶│   Footer Config  │
│                 │     │   (config)       │
└─────────────────┘     └──────────────────┘
```

## State Transitions

This feature has no dynamic state transitions. All configurations are static at build time.

## Data Sources

| Entity | Source File | Format |
|--------|-------------|--------|
| Module Cards | `src/components/ModuleCards/index.js` | JavaScript array |
| Theme Config | `src/css/custom.css` | CSS custom properties |
| Hero Section | `docusaurus.config.js` | JavaScript object |
| Navbar Items | `docusaurus.config.js` | JavaScript array |
| Footer Config | `docusaurus.config.js` | JavaScript object |
