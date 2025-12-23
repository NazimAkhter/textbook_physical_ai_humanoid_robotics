# Component Structure Contract: UI Update

**Feature**: `003-ui-book-update`
**Date**: 2024-12-15

## Overview

This contract defines the file structure and component interfaces for the UI update feature. Since this is a frontend-only Docusaurus project, contracts focus on component props, CSS class naming, and configuration structure rather than API endpoints.

## File Structure Contract

```text
frontend/
├── docusaurus.config.js          # MODIFY: title, tagline, navbar, footer
├── src/
│   ├── css/
│   │   └── custom.css            # MODIFY: theme colors
│   ├── pages/
│   │   ├── index.js              # MODIFY: hero section, import ModuleCards
│   │   └── index.module.css      # MODIFY: hero styling
│   └── components/
│       └── ModuleCards/          # CREATE: new component
│           ├── index.js          # Component logic
│           └── styles.module.css # Component styles
└── static/
    └── img/
        └── logo.svg              # OPTIONAL: update if new logo needed
```

## Component Contracts

### 1. ModuleCards Component

**File**: `src/components/ModuleCards/index.js`

**Props Interface**:
```typescript
interface ModuleCardProps {
  number: number;           // 1-5
  title: string;            // max 50 chars
  description: string;      // max 120 chars
  href: string;             // internal path, e.g., "/docs/module-01-..."
  status: "available" | "coming-soon";
}

interface ModuleCardsProps {
  modules: ModuleCardProps[];
}
```

**Exported Function**:
```javascript
export default function ModuleCards(): JSX.Element
```

**Behavior**:
- Renders a grid of module cards
- Cards with `status: "coming-soon"` show badge and are not clickable
- Responsive: 3 columns (desktop), 2 columns (tablet), 1 column (mobile)

### 2. Homepage Component Modification

**File**: `src/pages/index.js`

**Required Changes**:
```javascript
// ADD import
import ModuleCards from '@site/src/components/ModuleCards';

// MODIFY HomepageHeader
function HomepageHeader() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      {/* Updated title/tagline from siteConfig */}
      {/* CTA button linking to first module */}
    </header>
  );
}

// MODIFY Home default export
export default function Home() {
  return (
    <Layout>
      <HomepageHeader />
      <main>
        <ModuleCards />  {/* REPLACE HomepageFeatures */}
      </main>
    </Layout>
  );
}
```

## CSS Class Contracts

### Theme Variables (custom.css)

**Required Variables**:
```css
:root {
  --ifm-color-primary: #6366f1;
  --ifm-color-primary-dark: #4f46e5;
  --ifm-color-primary-darker: #4338ca;
  --ifm-color-primary-darkest: #3730a3;
  --ifm-color-primary-light: #818cf8;
  --ifm-color-primary-lighter: #a5b4fc;
  --ifm-color-primary-lightest: #c7d2fe;
}

[data-theme='dark'] {
  --ifm-color-primary: #818cf8;
  --ifm-color-primary-dark: #6366f1;
  --ifm-color-primary-darker: #4f46e5;
  --ifm-color-primary-darkest: #4338ca;
  --ifm-color-primary-light: #a5b4fc;
  --ifm-color-primary-lighter: #c7d2fe;
  --ifm-color-primary-lightest: #e0e7ff;
  --ifm-background-color: #1e1b4b;
}
```

### Module Cards Classes (styles.module.css)

**Required Classes**:
```css
.moduleCards { }        /* Grid container */
.moduleCard { }         /* Individual card */
.moduleCardAvailable { } /* Clickable card state */
.moduleCardComingSoon { } /* Disabled card state */
.moduleNumber { }       /* Module number badge */
.moduleTitle { }        /* Card title */
.moduleDescription { }  /* Card description */
.comingSoonBadge { }    /* "Coming Soon" indicator */
```

### Hero Section Classes (index.module.css)

**Required Modifications**:
```css
.heroBanner {
  /* Updated gradient/color using theme variables */
}

.buttons {
  /* CTA button container styling */
}
```

## Configuration Contracts

### docusaurus.config.js

**Required Changes**:

```javascript
const config = {
  title: 'Physical AI Book',
  tagline: 'Master humanoid robotics with ROS 2, digital twins, and Vision-Language-Action systems',

  // ... existing config ...

  themeConfig: {
    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'Physical AI Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          href: 'https://github.com/[owner]/[repo]',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            { label: 'Modules', to: '/docs/intro' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'GitHub', href: 'https://github.com/[owner]/[repo]' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book.`,
    },
  },
};
```

## Validation Checklist

### Build-Time Validation
- [ ] `npm run build` completes without errors
- [ ] No "broken links" warnings in build output
- [ ] All CSS classes resolve correctly

### Runtime Validation
- [ ] Homepage displays hero section with updated branding
- [ ] Module cards render in responsive grid
- [ ] All module card links navigate correctly
- [ ] Dark mode toggle functions correctly
- [ ] Mobile navigation hamburger menu works
- [ ] Footer displays correct content

### Accessibility Validation
- [ ] Color contrast ratio ≥ 4.5:1 for text
- [ ] All interactive elements are keyboard accessible
- [ ] Cards have appropriate ARIA labels
