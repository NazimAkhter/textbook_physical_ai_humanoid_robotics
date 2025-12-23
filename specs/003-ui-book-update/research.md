# Research: UI Update for Physical AI Book Website

**Feature**: `003-ui-book-update`
**Date**: 2024-12-15
**Status**: Complete

## Research Topics

### 1. Color Theme Palette Selection

**Decision**: Use a modern tech/AI-inspired color palette with deep violet primary and cyan accents.

**Rationale**:
- The reference image URL suggests a modern tech aesthetic
- Violet/purple is strongly associated with AI and machine learning branding
- Cyan/teal accents provide high contrast and complement violet
- This palette works well for both light and dark modes

**Chosen Colors**:
- **Primary (Light)**: `#6366f1` (Indigo-500 - modern violet)
- **Primary (Dark)**: `#818cf8` (Indigo-400 - lighter for dark mode contrast)
- **Accent**: `#06b6d4` (Cyan-500 - teal accent)
- **Background Dark**: `#1e1b4b` (Indigo-950 - deep purple-black)

**Alternatives Considered**:
- Green/teal primary (Docusaurus default) - Rejected: too generic, doesn't convey AI theme
- Blue primary - Rejected: overused, less distinctive
- Orange/amber - Rejected: doesn't align with AI/robotics aesthetic

### 2. Module Card Component Pattern

**Decision**: Create a dedicated `ModuleCard` React component with grid layout using CSS Grid.

**Rationale**:
- CSS Grid provides native responsive layout without complex JavaScript
- Component encapsulation follows React/Docusaurus best practices
- Allows easy addition of future modules by extending the data array

**Pattern**:
```
ModuleCard props: { number, title, description, href, status }
- status: "available" | "coming-soon"
- Grid: 3 columns (>1024px), 2 columns (768-1024px), 1 column (<768px)
```

**Alternatives Considered**:
- Flexbox only - Rejected: more complex for responsive grid
- Docusaurus plugin - Rejected: overkill for static card list
- Hardcoded HTML - Rejected: not maintainable

### 3. Hero Section Implementation

**Decision**: Modify existing `HomepageHeader` component in `src/pages/index.js`.

**Rationale**:
- Existing component provides the structure
- Docusaurus `siteConfig` already provides title/tagline hooks
- Minimal changes required - update styling and CTA link

**Implementation Approach**:
- Update `docusaurus.config.js` for site title and tagline
- Modify hero CSS in `index.module.css`
- Update CTA button link to point to first module

**Alternatives Considered**:
- Custom hero component - Rejected: unnecessary complexity
- Third-party hero library - Rejected: adds dependencies

### 4. Navigation Bar Configuration

**Decision**: Configure navbar via `docusaurus.config.js` themeConfig.

**Rationale**:
- Docusaurus provides declarative navbar configuration
- No custom components needed
- Follows Docusaurus patterns for maintainability

**Navbar Items**:
1. Logo + "Physical AI Book" title (left)
2. "Modules" dropdown linking to documentation (left)
3. GitHub link (right)

**Alternatives Considered**:
- Custom navbar component - Rejected: config approach is simpler
- Multiple top-level nav items - Rejected: clutters navigation

### 5. Broken Link Detection Strategy

**Decision**: Use Docusaurus built-in `onBrokenLinks: 'throw'` + manual audit.

**Rationale**:
- Docusaurus already has `onBrokenLinks: 'throw'` configured
- Build process will fail on broken internal links
- Manual audit catches configuration-level issues

**Audit Checklist**:
1. Remove default Docusaurus tutorial links from config
2. Update footer links to project-relevant destinations
3. Verify hero CTA points to valid module path
4. Test all navbar items post-update

**Alternatives Considered**:
- External link checker tool - Deferred: not needed for this scope
- Automated CI link checking - Deferred: can add in future iteration

### 6. Footer Content Strategy

**Decision**: Simplify footer to single column with essential links only.

**Rationale**:
- Default Docusaurus footer has placeholder content (Stack Overflow, Discord, etc.)
- Educational book doesn't need community links initially
- Simpler footer reduces maintenance burden

**Footer Structure**:
- Single section with: Documentation link, GitHub link
- Copyright with project name
- Remove: Community section, Blog link, external social links

**Alternatives Considered**:
- Multi-column footer - Deferred: can expand when needed
- No footer - Rejected: maintains professional appearance

### 7. Responsive Breakpoints

**Decision**: Use standard breakpoints aligned with Docusaurus/Infima defaults.

**Rationale**:
- Docusaurus uses Infima CSS framework with established breakpoints
- Matching these prevents CSS conflicts
- Industry standard breakpoints cover common devices

**Breakpoints**:
- Mobile: < 768px (single column)
- Tablet: 768px - 996px (2 columns for cards)
- Desktop: > 996px (3 columns for cards)

Note: Docusaurus uses 996px as tablet/desktop breakpoint, not 1024px.

**Alternatives Considered**:
- Custom breakpoints - Rejected: would conflict with Infima

### 8. Dark Mode Implementation

**Decision**: Use CSS custom properties in `custom.css` with `[data-theme='dark']` selector.

**Rationale**:
- Docusaurus dark mode toggle is built-in
- CSS custom properties provide easy theming
- Pattern already exists in default `custom.css`

**Color Adjustments for Dark Mode**:
- Primary color lightened for visibility
- Background uses deep indigo instead of pure black
- Text contrast verified for WCAG AA (4.5:1)

**Alternatives Considered**:
- Separate dark mode stylesheet - Rejected: more complex
- JavaScript theme switching - Rejected: CSS is sufficient

## Technology Stack Confirmation

| Component | Technology | Version |
|-----------|------------|---------|
| Framework | Docusaurus | 3.x |
| Styling | CSS (Infima + custom) | N/A |
| Components | React (JSX) | 18.x |
| Deployment | Vercel | N/A |
| Testing | Manual + Build validation | N/A |

## Dependencies

No new dependencies required. All changes use existing Docusaurus/React capabilities.

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Color palette doesn't match reference | Medium | Document specific hex values; easy to adjust |
| Responsive layout breaks | High | Test at all breakpoints before PR |
| Breaking existing links | High | Use `onBrokenLinks: 'throw'`; manual audit |
| Dark mode contrast issues | Medium | Verify WCAG AA with contrast checker |
