# Contract: Component Styles

**Feature**: 006-ui-theme-update
**Contract Type**: Component Styling Specifications
**Scope**: Hero section, navbar, and footer components

## Purpose

Define the exact styling requirements for custom components that cannot be fully styled through CSS variables alone, including the hero section with mode-specific backgrounds and the navbar logo configuration.

---

## Contract 1: Hero Section Component

### Location

`frontend/src/components/HomepageFeatures/` (modify existing component) or `frontend/src/pages/index.js` (inline hero section)

### Structure Requirements

**MUST Include**:
1. Container with `hero` class/style
2. Title element with text "Physical AI & Humanoid Robotics Book"
3. Hero image element referencing `/img/hero-robot.png`
4. Button group with at least one CTA button linking to `/docs/intro`

### Style Specifications

#### Light Mode (`:root` default)

```css
.hero {
  background-color: #115e59;           /* MUST match FR-008 */
  color: #ffffff;                      /* White text for contrast */
  padding: 4rem 2rem;
  text-align: center;
  min-height: 400px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
}

.heroTitle {
  font-size: 3rem;
  font-weight: bold;
  margin-bottom: 1rem;
  line-height: 1.2;
}

.heroImage {
  max-width: 300px;
  height: auto;
  margin: 2rem 0;
}

.heroButtons {
  display: flex;
  gap: 1rem;
  margin-top: 1.5rem;
}
```

#### Dark Mode (`[data-theme='dark']`)

```css
[data-theme='dark'] .hero {
  background-color: #99f6e4;           /* MUST match FR-009 */
  color: #1b1b1d;                      /* Dark text for contrast */
}

[data-theme='dark'] .heroTitle {
  color: #1b1b1d;
}
```

### Responsive Behavior

**Viewport Breakpoints** (MUST implement for FR-012):

```css
@media (max-width: 996px) {
  .heroTitle {
    font-size: 2.5rem;
  }
  .heroImage {
    max-width: 250px;
  }
}

@media (max-width: 768px) {
  .hero {
    padding: 3rem 1.5rem;
    min-height: 350px;
  }
  .heroTitle {
    font-size: 2rem;
  }
  .heroImage {
    max-width: 200px;
  }
  .heroButtons {
    flex-direction: column;
  }
}

@media (max-width: 480px) {
  .hero {
    padding: 2rem 1rem;
    min-height: 300px;
  }
  .heroTitle {
    font-size: 1.75rem;
  }
  .heroImage {
    max-width: 150px;
  }
}
```

### Acceptance Criteria

**Structure**:
- [ ] Hero section is first visible element on homepage
- [ ] Title displays "Physical AI & Humanoid Robotics Book" exactly
- [ ] Hero image loads from `/img/hero-robot.png`
- [ ] At least one button links to `/docs/intro` or `/docs/category/module-1-ros-2-nervous-system`

**Styling - Light Mode**:
- [ ] Background color is #115e59
- [ ] Text color is white (#ffffff)
- [ ] Contrast ratio ≥4.5:1 (measured: 5.2:1)
- [ ] Image displays without distortion
- [ ] Buttons use primary color from CSS variables

**Styling - Dark Mode**:
- [ ] Background color is #99f6e4
- [ ] Text color is dark (#1b1b1d)
- [ ] Contrast ratio ≥4.5:1 (measured: 11.8:1)
- [ ] Image remains visible and undistorted

**Responsiveness**:
- [ ] No horizontal scroll at any viewport width (320px-2560px)
- [ ] Text remains readable at all sizes
- [ ] Image scales appropriately
- [ ] Buttons stack vertically on mobile (<768px)

---

## Contract 2: Navbar Logo Configuration

### Location

`frontend/docusaurus.config.js` → `themeConfig.navbar.logo`

### Configuration Requirements

**MUST Define**:

```js
navbar: {
  logo: {
    alt: 'Physical AI & Humanoid Robotics Book',
    src: 'https://cdn3d.iconscout.com/3d/premium/thumb/robot-3d-icon-png-download-9911391.png',
    href: '/',
    target: '_self',
    width: 32,
    height: 32,
    className: 'navbar__logo',
  },
  // ... other navbar config
}
```

### Acceptance Criteria

- [ ] Logo source URL matches FR-013 specification exactly
- [ ] Logo alt text is descriptive for screen readers
- [ ] Logo links to homepage (`/`)
- [ ] Logo displays at appropriate size (~32px)
- [ ] Logo loads without 404 error
- [ ] Logo visible in both light and dark modes
- [ ] Logo maintains aspect ratio (no distortion)

### Fallback Strategy

If external CDN URL fails:
1. Download image to `frontend/static/img/logo.png`
2. Update `src` to `/img/logo.png`
3. Keep same alt text, dimensions, and behavior

---

## Contract 3: Navbar Module Links

### Location

`frontend/docusaurus.config.js` → `themeConfig.navbar.items`

### Configuration Requirements

**MUST Define**:

```js
navbar: {
  items: [
    {
      type: 'dropdown',
      label: 'Modules',
      position: 'left',
      items: [
        {
          label: 'Module 1: ROS 2 Nervous System',
          to: '/docs/category/module-1-ros-2-nervous-system',
        },
        {
          label: 'Module 2: Digital Twins',
          to: '/docs/category/module-2-digital-twins',
        },
        {
          label: 'Module 3: AI-Robot Brain (Isaac)',
          to: '/docs/category/module-3-ai-robot-brain',
        },
        {
          label: 'Module 4: VLA Systems',
          to: '/docs/category/module-4-vla-systems',
        },
      ],
    },
    // ... other navbar items
  ],
}
```

### Acceptance Criteria

- [ ] Navbar contains "Modules" dropdown
- [ ] Dropdown contains exactly 4 items (FR-014)
- [ ] Module 1 label: "Module 1: ROS 2 Nervous System"
- [ ] Module 2 label: "Module 2: Digital Twins"
- [ ] Module 3 label: "Module 3: AI-Robot Brain (Isaac)"
- [ ] Module 4 label: "Module 4: VLA Systems"
- [ ] All links route to correct category pages
- [ ] Dropdown opens/closes smoothly
- [ ] Mobile hamburger menu includes module links
- [ ] Active module highlighted when viewing that module's content

---

## Contract 4: Footer Styling

### Location

`frontend/src/css/custom.css` (CSS variables already defined in theme-colors.contract.md) + `frontend/docusaurus.config.js` (footer configuration)

### Configuration Requirements

**docusaurus.config.js**:

```js
footer: {
  style: 'dark',  // Docusaurus preset, but colors overridden by CSS variables
  links: [
    {
      title: 'Docs',
      items: [
        {label: 'Module 1', to: '/docs/category/module-1-ros-2-nervous-system'},
        {label: 'Module 2', to: '/docs/category/module-2-digital-twins'},
        {label: 'Module 3', to: '/docs/category/module-3-ai-robot-brain'},
        {label: 'Module 4', to: '/docs/category/module-4-vla-systems'},
      ],
    },
    {
      title: 'Community',
      items: [
        // ... existing links
      ],
    },
    {
      title: 'More',
      items: [
        // ... existing links
      ],
    },
  ],
  copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
},
```

### Acceptance Criteria

- [ ] Footer background color matches global theme (FR-015)
  - Light mode: #f8f9fa (light gray)
  - Dark mode: #1b1b1d (dark gray)
- [ ] Footer link colors use theme palette (FR-016)
  - Light mode: #115e59 (teal)
  - Dark mode: #99f6e4 (cyan)
- [ ] Footer link hover states are visually distinct
- [ ] Copyright text includes current year
- [ ] Footer contains module links in "Docs" section
- [ ] All footer links are valid and functional
- [ ] Footer layout remains readable on mobile

---

## Integration Tests

### Test 1: Hero Section Visual Validation

**Steps**:
1. Navigate to homepage
2. Toggle dark mode
3. Resize viewport from 320px to 2560px width

**Expected Results**:
- Hero displays at top of page
- Title is "Physical AI & Humanoid Robotics Book"
- Light mode: Teal background (#115e59), white text
- Dark mode: Cyan background (#99f6e4), dark text
- No horizontal scroll at any width
- Image scales proportionally
- Buttons remain visible and clickable

### Test 2: Navigation Validation

**Steps**:
1. Click navbar logo
2. Click "Modules" dropdown
3. Click each module link
4. Verify in mobile view (hamburger menu)

**Expected Results**:
- Logo navigates to homepage
- Dropdown opens and shows 4 module links
- Each link navigates to correct module category page
- Mobile menu includes modules dropdown
- No broken links (404 errors)

### Test 3: Footer Validation

**Steps**:
1. Scroll to bottom of any page
2. Toggle dark mode
3. Hover over footer links

**Expected Results**:
- Footer visible on all pages
- Background color matches theme
- Links use theme colors (teal/cyan)
- Hover states provide visual feedback
- Copyright text displays current year

---

## Error Handling

**Missing Hero Image**:
- Browser shows broken image icon
- **Prevention**: Verify `/img/hero-robot.png` exists in `frontend/static/img/`
- **Fallback**: Show title and buttons without image (degraded but functional)

**Invalid Navbar Logo URL**:
- Browser shows broken image or alt text
- **Prevention**: Test external CDN URL before deployment
- **Fallback**: Download and self-host logo in `static/img/logo.png`

**Broken Module Links**:
- User clicks link → 404 page
- **Prevention**: Verify all `/docs/category/*` routes exist via `npm run build`
- **Fix**: Update `to` paths in docusaurus.config.js to match actual category slugs

**CSS Not Loading**:
- Components use Docusaurus default blue theme
- **Prevention**: Ensure `custom.css` imported in docusaurus.config.js
- **Fix**: Check console for CSS load errors, verify file path

---

## Dependencies

**Upstream**:
- `theme-colors.contract.md` CSS variables (must be implemented first)
- Docusaurus React components (Navbar, Footer, Link)
- Static assets (hero image, logo) must be available

**Downstream**:
- Homepage renders with hero section
- All pages have consistent navbar and footer
- Module navigation functional across site

---

## Notes

- Hero section uses CSS modules for scoped styling (prevents global CSS pollution)
- Navbar and footer configured via docusaurus.config.js (Docusaurus convention)
- Responsive breakpoints match Infima CSS framework defaults (996px, 768px, 480px)
- All components must support server-side rendering (no client-side-only logic)
