# Implementation Plan: UI Update for Physical AI Book Website

**Branch**: `003-ui-book-update` | **Date**: 2024-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-ui-book-update/spec.md`

## Summary

Modernize the Physical AI Book website's user interface with updated homepage hero section, module cards navigation, professional color theme (indigo/violet primary with cyan accents), and fixed navigation links. The implementation uses existing Docusaurus 3.x patterns with CSS custom properties for theming and React components for module cards.

## Technical Context

**Language/Version**: JavaScript/JSX (ES2020+), CSS3
**Primary Dependencies**: Docusaurus 3.x, React 18.x, Infima CSS Framework
**Storage**: N/A (static site generation)
**Testing**: Manual testing + Docusaurus build validation (`onBrokenLinks: 'throw'`)
**Target Platform**: Web (Vercel deployment), mobile-responsive
**Project Type**: Web application (frontend only - Docusaurus documentation site)
**Performance Goals**: < 3 second page load, 60fps scrolling
**Constraints**: Must use Docusaurus configuration patterns, no external UI libraries
**Scale/Scope**: Single homepage update, 5 module cards, site-wide theme

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-Driven Development | ✅ PASS | Following `/sp.specify` → `/sp.clarify` → `/sp.plan` sequence |
| II. Educational Efficacy | ✅ PASS | UI improvements enhance access to educational content |
| III. Documentation–Agent Integration | ⚠️ N/A | This feature doesn't modify chatbot or RAG |
| IV. Reproducibility by Default | ⚠️ N/A | UI feature, no code examples affected |
| V. Technical Standards | ✅ PASS | Using Docusaurus + Vercel per constitution |
| VI. Constraints & Compliance | ✅ PASS | Claude Code + Spec-Kit Plus only |

**Note**: Principles III and IV are not applicable to this UI-only feature. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/003-ui-book-update/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Research decisions (color palette, components)
├── data-model.md        # UI entity definitions
├── quickstart.md        # Implementation guide
├── contracts/           # Component structure contracts
│   └── component-structure.md
├── checklists/          # Validation checklists
│   └── requirements.md
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
frontend/
├── docusaurus.config.js      # MODIFY: title, tagline, navbar, footer
├── src/
│   ├── css/
│   │   └── custom.css        # MODIFY: theme colors (light/dark)
│   ├── pages/
│   │   ├── index.js          # MODIFY: hero section, import ModuleCards
│   │   └── index.module.css  # MODIFY: hero banner styling
│   └── components/
│       ├── HomepageFeatures/ # DELETE: replaced by ModuleCards
│       └── ModuleCards/      # CREATE: new component
│           ├── index.js      # Card component logic + data
│           └── styles.module.css # Responsive grid styles
└── static/
    └── img/                  # Optional: logo update if needed
```

**Structure Decision**: Frontend-only modification. Using existing Docusaurus project structure with new ModuleCards component in `src/components/`.

## Architecture Sketch

### Component Hierarchy

```
Layout (Docusaurus)
├── Navbar (config-driven)
│   ├── Logo + Title
│   └── Nav Items (Modules, GitHub)
├── Main Content
│   ├── HomepageHeader (hero)
│   │   ├── Title (from siteConfig)
│   │   ├── Tagline (from siteConfig)
│   │   └── CTA Button → Module 1
│   └── ModuleCards (new component)
│       ├── ModuleCard (Module 1) → /docs/module-01-...
│       ├── ModuleCard (Module 2) → /docs/module-02-...
│       ├── ModuleCard (Module 3) → Coming Soon
│       ├── ModuleCard (Module 4) → Coming Soon
│       └── ModuleCard (Module 5) → Coming Soon
└── Footer (config-driven)
    ├── Learn section
    └── More section
```

### Theme Flow

```
custom.css
├── :root (light mode)
│   └── --ifm-color-primary: #6366f1 (indigo)
└── [data-theme='dark']
    ├── --ifm-color-primary: #818cf8 (lighter indigo)
    └── --ifm-background-color: #1e1b4b (deep purple)
```

### Responsive Breakpoints

```
Desktop (>996px)     Tablet (768-996px)    Mobile (<768px)
┌───┬───┬───┐        ┌───┬───┐             ┌───┐
│ 1 │ 2 │ 3 │        │ 1 │ 2 │             │ 1 │
├───┼───┼───┤        ├───┼───┤             ├───┤
│ 4 │ 5 │   │        │ 3 │ 4 │             │ 2 │
└───┴───┴───┘        ├───┼───┤             ├───┤
                     │ 5 │   │             │...│
                     └───┴───┘             └───┘
```

## Phased Execution Plan

### Phase 1: Theme & Configuration (US2, US3)

1. Update `src/css/custom.css` with new color palette
2. Update `docusaurus.config.js`:
   - Site title and tagline
   - Navbar configuration
   - Footer configuration
3. Verify build passes with theme changes

### Phase 2: Module Cards Component (US1 - MVP)

4. Create `src/components/ModuleCards/` directory
5. Create `ModuleCards/index.js` with component and data
6. Create `ModuleCards/styles.module.css` with responsive grid
7. Update `src/pages/index.js` to use ModuleCards
8. Update hero section CTA link

### Phase 3: Cleanup & Validation (US4)

9. Remove or archive `HomepageFeatures` component
10. Audit all navigation links
11. Run full build validation
12. Test responsive layouts
13. Verify dark mode contrast

## Testing & Validation Strategy

### Build-Time Validation

| Check | Method | Pass Criteria |
|-------|--------|---------------|
| Broken links | `npm run build` | 0 broken link warnings |
| Build success | `npm run build` | Exit code 0 |
| CSS compilation | Build output | No CSS errors |

### Manual Testing

| Check | Method | Pass Criteria |
|-------|--------|---------------|
| Hero section | Visual inspection | Correct title, tagline, CTA |
| Module cards | Click each card | Navigates to correct module |
| Responsive (375px) | DevTools mobile | Single column, no overflow |
| Responsive (768px) | DevTools tablet | Two columns |
| Responsive (1440px) | DevTools desktop | Three columns |
| Dark mode | Toggle theme | Colors change, contrast OK |
| Navbar | Click all items | All links resolve |
| Footer | Click all items | All links resolve |

### Accessibility Testing

| Check | Tool | Pass Criteria |
|-------|------|---------------|
| Color contrast | Browser extension | 4.5:1 ratio minimum |
| Keyboard nav | Tab through page | All elements reachable |

## Decisions & Tradeoffs

### Decision 1: Color Palette

**Options Considered**:
- A: Keep Docusaurus default (green)
- B: Blue/tech theme
- C: Indigo/violet AI theme (Selected)

**Selected**: Option C - Indigo/violet primary with cyan accents

**Rationale**: Aligns with AI/ML branding conventions, provides distinctive look, works well in both light and dark modes.

### Decision 2: Module Cards vs Docusaurus Features

**Options Considered**:
- A: Modify existing HomepageFeatures
- B: Create new ModuleCards component (Selected)
- C: Use third-party card library

**Selected**: Option B - Create new ModuleCards component

**Rationale**: HomepageFeatures has SVG-based layout not suitable for module navigation. New component provides cleaner separation and proper linking semantics.

### Decision 3: Footer Simplification

**Options Considered**:
- A: Keep multi-column with community links
- B: Simplify to essential links only (Selected)
- C: Remove footer entirely

**Selected**: Option B - Simplify to essential links

**Rationale**: Educational book doesn't need community Discord/Stack Overflow links. Simpler footer reduces maintenance and focuses on content.

## Decisions Requiring ADR

No significant architectural decisions requiring ADR were identified. All changes follow established Docusaurus patterns.

## Complexity Tracking

> No constitution violations requiring justification.

This feature follows standard Docusaurus patterns with minimal custom code. The only new component (ModuleCards) is a simple presentational component with static data.

## Next Steps

1. Run `/sp.tasks` to generate implementation task list
2. Execute Phase 1 (Theme & Configuration)
3. Execute Phase 2 (Module Cards Component)
4. Execute Phase 3 (Cleanup & Validation)
5. Create PR for review
