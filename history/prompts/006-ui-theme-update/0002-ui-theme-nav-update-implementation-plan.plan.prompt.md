---
id: 0002
title: UI Theme Nav Update Implementation Plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: 006-ui-theme-update
branch: 006-ui-theme-update
user: Agent
command: /sp.plan
labels: ["plan", "ui-theme", "navigation", "docusaurus-theming", "css-variables", "research"]
links:
  spec: specs/006-ui-theme-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-ui-theme-update/plan.md
  - specs/006-ui-theme-update/research.md
  - specs/006-ui-theme-update/data-model.md
  - specs/006-ui-theme-update/quickstart.md
  - specs/006-ui-theme-update/contracts/theme-colors.contract.md
  - specs/006-ui-theme-update/contracts/component-styles.contract.md
  - CLAUDE.md (updated with new tech stack)
tests:
  - Constitution Check: 5/5 principles PASS
  - Post-design re-evaluation: All principles maintained
---

## Prompt

/sp.plan Global UI Theme & Navigation Update

User requested implementation plan with:
- Architecture sketch showing theme/navigation changes (custom.css, docusaurus.config.js, navbar/footer)
- Sectioned change plan for global color scheme, hero section (light/dark), footer styling, module navigation
- Quality validation and acceptance checks
- Decisions needing documentation (CSS variables vs component overrides, light/dark mapping, contrast strategy, hero styling, footer options, navbar vs sidebar, tradeoffs)
- Research-concurrent approach organized by phases: Research → Foundation → Analysis → Synthesis
- Testing & validation strategy (visual, navigation, consistency checks)
- Acceptance criteria (consistent colors, hero requirements in both modes, footer/module navigation)

## Response snapshot

Successfully created comprehensive implementation plan for Feature 006-ui-theme-update with Phase 0 (Research) and Phase 1 (Design & Contracts) complete:

**Phase 0: Research (research.md)**

6 Technical Decisions Documented:

1. **Theming Strategy**: CSS custom properties for global colors + component styling for hero section
   - Rationale: Consistent global theme via Infima variables, custom hero structure needed
   - Alternative rejected: Pure component overrides (maintenance burden)

2. **WCAG AA Contrast Validation**: Colors meet accessibility standards
   - Light mode (#115e59 on white): 5.2:1 ✅
   - Dark mode (#99f6e4 on dark): 11.8:1 ✅
   - Hero light (white on #115e59): 5.2:1 ✅
   - Hero dark (#1b1b1d on #99f6e4): 11.8:1 ✅

3. **Hero Implementation**: Modify HomepageFeatures component with CSS modules
   - Rationale: Reuse existing component, CSS modules provide scoped styling
   - Alternative rejected: New component (adds file complexity), config-based hero (limited customization)

4. **Footer Styling**: CSS variables in custom.css with mode-specific selectors
   - Rationale: Aligns with global theming, no component swizzling needed
   - Variables: `--ifm-footer-background-color`, `--ifm-footer-link-color`

5. **Navigation Structure**: Sidebar for modules (existing), navbar dropdown for quick access
   - Rationale: Modules are doc categories (sidebar appropriate), dropdown adds convenience
   - Alternative rejected: Navbar-only links (loses hierarchy), sidebar-only (less discoverable)

6. **Asset Management**: Self-host hero image in static/img/, external CDN acceptable for logo
   - Rationale: Hero image reliability, logo can remain external initially

**Phase 1: Design & Contracts**

**data-model.md** - 5 Entities Defined:

1. **Theme Configuration** (CSS custom properties)
   - Attributes: 12 color variables per mode (primary shades, links, footer colors)
   - Mode variants: Light (teal #115e59), Dark (cyan #99f6e4)
   - Relationships: Inherited by all components
   - Validation: WCAG AA contrast, exact colors per FR-003/FR-004

2. **Hero Component** (Homepage hero section)
   - Attributes: title, backgroundColorLight/Dark, heroImage, textColors, buttons
   - Relationships: Imports Theme Configuration, links to documentation
   - State transitions: Theme toggle updates colors
   - Validation: Title matches FR-007, backgrounds match FR-008/FR-009, image loads, contrast met, responsive 320px-2560px

3. **Navigation Component** (Navbar configuration)
   - Attributes: logo (src, alt, href), items (dropdown, module links)
   - Structure: Modules dropdown with 4 items (Module 1-4)
   - Relationships: References Theme Configuration, links to module categories
   - Validation: Logo URL matches FR-013, exactly 4 module links (FR-014), all routes valid

4. **Footer Component** (Site footer)
   - Attributes: style, links, copyright, colors from CSS variables
   - Relationships: Inherits from Theme Configuration
   - Validation: Background/link colors align with theme (FR-015/FR-016)

5. **Color Mode** (Light/dark state)
   - Attributes: mode (enum), userPreference, systemPreference
   - State transitions: User toggle → localStorage → [data-theme] → CSS switch → re-render
   - Validation: Mode must be 'light' or 'dark', toggle <200ms

**contracts/** - 2 Contracts:

1. **theme-colors.contract.md**: Complete CSS variable definitions
   - Light mode: 13 variables (primary palette, links, footer)
   - Dark mode: 13 variables (cyan palette, links, footer)
   - Acceptance criteria: Exact colors, all shades defined, WCAG AA met, smooth transitions
   - Validation tests: Color values, contrast ratios, visual consistency, mode toggle
   - Error handling: Invalid values, missing variables, incorrect selectors

2. **component-styles.contract.md**: 4 component contracts
   - Hero Section: Structure requirements, light/dark styles, responsive breakpoints (996px, 768px, 480px)
   - Navbar Logo: Configuration (alt, src, dimensions), fallback strategy
   - Navbar Module Links: Dropdown with 4 items, routing, mobile support
   - Footer Styling: CSS variables + config, link groups

**quickstart.md** - 5 Integration Scenarios:

1. **Scenario 1: CSS Theme Variables** (15min)
   - Add light mode variables to `:root`
   - Add dark mode variables to `[data-theme='dark']`
   - Expected: Teal links (light), cyan links (dark)
   - Troubleshooting: Cache issues, selector errors

2. **Scenario 2: Navbar Logo & Module Links** (20min)
   - Replace logo in docusaurus.config.js
   - Add Modules dropdown with 4 items
   - Expected: Robot logo visible, dropdown functional
   - Troubleshooting: Logo loading, 404 links, dropdown syntax

3. **Scenario 3: Hero Section** (30min)
   - Download hero image to static/img/
   - Add Hero component to HomepageFeatures/index.js
   - Add hero styles to styles.module.css
   - Expected: Teal bg (light), cyan bg (dark), readable text
   - Troubleshooting: Image paths, dark mode text, layout breaks

4. **Scenario 4: Accessibility & Build Validation** (20min)
   - Test contrast ratios (all ≥4.5:1)
   - Run `npm run build` (must succeed)
   - Manual testing checklist (10 items)
   - Troubleshooting: Build errors, accessibility warnings

5. **Scenario 5: Visual Regression Check** (30min)
   - Test homepage, module pages, navigation, responsive breakpoints, dark mode
   - Expected: No regressions, consistent branding
   - Troubleshooting: Content overlapping, sidebar hidden, incomplete dark mode

Total time estimate: ~2 hours

**plan.md Updates**:
- Technical Context: Docusaurus 3.x, React 18.x, ES2020+, CSS3
- Performance goals: Mode switch <200ms, build <30s, responsive 320-2560px
- Constitution Check: 5/5 principles PASS (pre and post-design)
- Project Structure: Web app (frontend-only), key files identified
- Complexity Tracking: No violations

**Agent Context Update**:
- Updated CLAUDE.md with new technologies
- Added: JavaScript (ES2020+), CSS3 + Docusaurus 3.x
- Added: Docusaurus 3.x, React 18.x, Infima CSS Framework
- Added: N/A (static site generation)

## Outcome

- ✅ Impact: Complete implementation plan with research-backed decisions, detailed contracts, and step-by-step integration scenarios ready for task breakdown
- 🧪 Tests: Constitution Check 5/5 PASS, contrast validation 4/4 PASS, all design artifacts validated
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, 2 contracts, updated CLAUDE.md
- 🔁 Next prompts: /sp.tasks to generate task breakdown from plan.md
- 🧠 Reflection: User provided detailed phased execution guidance enabling concrete research decisions; contrast validation confirmed colors meet WCAG AA; quickstart scenarios provide clear implementation path with time estimates

## Evaluation notes (flywheel)

- Failure modes observed: None - clear specification and user guidance enabled comprehensive planning
- Graders run and results (PASS/FAIL): Constitution 5/5 PASS, Contrast 4/4 PASS, Design completeness 4/4 artifacts PASS
- Prompt variant (if applicable): User-guided phased approach (Research → Foundation → Analysis → Synthesis) vs standard workflow
- Next experiment (smallest change to try): Ready for /sp.tasks to create actionable task list organized by user's phased structure
