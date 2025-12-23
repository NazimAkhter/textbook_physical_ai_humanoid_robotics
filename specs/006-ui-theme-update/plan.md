# Implementation Plan: UI Theme & Navigation Update

**Branch**: `006-ui-theme-update` | **Date**: 2025-12-16 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/006-ui-theme-update/spec.md`

**Note**: This plan follows the `/sp.plan` command workflow with research-concurrent approach.

## Summary

Apply a unified teal/cyan color scheme across the Physical AI & Humanoid Robotics Book Docusaurus site, updating hero section styling for light/dark modes, improving footer styling, and restructuring navigation to module-based links (Module 1-4). Technical approach uses Docusaurus CSS custom properties for theme variables, component-level styling for hero section, and docusaurus.config.js modifications for navbar/footer configuration. Implementation follows phased approach: Research (Docusaurus theming patterns, accessibility validation) → Foundation (CSS variables, navigation structure) → Analysis (hero/footer styling) → Synthesis (normalization, consistency checks).

## Technical Context

**Language/Version**: JavaScript (ES2020+), CSS3 + Docusaurus 3.x
**Primary Dependencies**: Docusaurus 3.x, React 18.x, Infima CSS Framework
**Storage**: N/A (static site generation)
**Testing**: Manual visual validation (light/dark modes), accessibility testing (WCAG AA contrast), Docusaurus build validation
**Target Platform**: Web (modern browsers: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
**Project Type**: Web application (frontend-only Docusaurus static site)
**Performance Goals**: Mode switching <200ms, build time <30s, responsive 320px-2560px viewports
**Constraints**: WCAG AA contrast ratios (4.5:1 body text, 3:1 large text), no horizontal scroll, zero build errors, no content changes
**Scale/Scope**: 4 modules, ~20 documentation pages, 2 color modes (light/dark), 3 main components (hero, navbar, footer)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Spec-Driven Development** ✅ PASS
- Workflow followed: `/sp.specify` → `/sp.plan` → (next: `/sp.tasks` → `/sp.implement`)
- Specification complete and validated (16/16 checklist items passed)

**Principle II: Educational Efficacy** ✅ PASS (Not Applicable - UI Feature)
- Feature is UI/theme update, not educational content
- Does not affect ROS 2, Digital Twins, Isaac, or VLA content
- Maintains existing content structure unchanged (FR-020)

**Principle III: Seamless Documentation–Agent Integration** ✅ PASS
- UI changes do not affect RAG chatbot integration
- Book content remains single source of truth
- No changes to content stored in Qdrant

**Principle IV: Reproducibility by Default** ✅ PASS (Not Applicable - UI Feature)
- No code examples affected by UI theme changes
- Existing code remains syntactically valid

**Principle V: Technical Standards** ✅ PASS
- Frontend: Docusaurus (latest stable 3.x) ✅
- Design: Mobile-responsive UI maintained (320px-2560px) ✅
- Deployment: Vercel hosting unchanged ✅

**Post-Design Re-check Status**: ✅ RE-EVALUATED - ALL PRINCIPLES PASS

After completing Phase 0 (Research) and Phase 1 (Design & Contracts):
- **Principle I**: ✅ PASS - Workflow maintained
- **Principle II**: ✅ PASS - No educational content affected
- **Principle III**: ✅ PASS - RAG integration unaffected
- **Principle IV**: ✅ PASS - No code examples modified
- **Principle V**: ✅ PASS - Docusaurus 3.x confirmed, responsive design maintained

**Artifacts Created**:
- research.md: 6 decisions documented (theming approach, contrast validation, hero implementation, footer styling, navigation structure, asset management)
- data-model.md: 5 entities defined (Theme Configuration, Hero Component, Navigation Component, Footer Component, Color Mode)
- contracts/: 2 contracts (theme-colors.contract.md with CSS variable specifications, component-styles.contract.md with hero/navbar/footer contracts)
- quickstart.md: 5 integration scenarios (CSS setup, navbar config, hero creation, validation, regression testing)

**No new concerns identified** - Design phase confirms straightforward CSS/config-based implementation

## Project Structure

### Documentation (this feature)

```text
specs/006-ui-theme-update/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (Docusaurus theming research)
├── data-model.md        # Phase 1 output (Theme configuration entities)
├── quickstart.md        # Phase 1 output (Implementation scenarios)
├── contracts/           # Phase 1 output (CSS variable contracts)
│   ├── theme-colors.contract.md
│   └── component-styles.contract.md
├── checklists/
│   └── requirements.md  # Validation checklist (16/16 PASS)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created yet)
```

### Source Code (repository root)

```text
frontend/
├── docusaurus.config.js       # MODIFY: Navbar/footer config, theme settings
├── src/
│   ├── css/
│   │   └── custom.css         # MODIFY: CSS custom properties for theme
│   ├── components/
│   │   ├── HomepageFeatures/  # MODIFY: Hero section component
│   │   │   ├── index.js
│   │   │   └── styles.module.css
│   │   ├── ModuleCards/       # EXISTS: No changes needed (already updated)
│   │   │   ├── index.js
│   │   │   └── styles.module.css
│   │   └── Footer/            # CHECK: May need custom footer component
│   │       ├── index.js
│   │       └── styles.module.css
│   └── pages/
│       └── index.js           # MODIFY: Homepage with hero section
├── static/
│   └── img/
│       ├── logo.png           # REPLACE: New robot logo
│       └── hero-robot.png     # ADD: Hero section robot image
└── sidebars.js                # REVIEW: Module navigation structure
```

**Structure Decision**: Web application (frontend-only Docusaurus static site). This feature modifies existing Docusaurus configuration and styling files. Primary changes are in `docusaurus.config.js` (navigation/theme config), `src/css/custom.css` (CSS variables), hero component styling (JSX + CSS modules), and static assets (logo/hero images). No backend changes required as this is purely a visual/theming update.

**Key Files to Modify**:
1. `frontend/docusaurus.config.js` - Navbar items (Module 1-4 links), footer config, theme color overrides
2. `frontend/src/css/custom.css` - CSS custom properties for teal/cyan palette (light/dark modes)
3. `frontend/src/pages/index.js` or `frontend/src/components/HomepageFeatures/` - Hero section title, background, buttons
4. `frontend/static/img/` - Add new logo and hero image assets

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected** - All constitution principles pass. This is a straightforward UI theming update using standard Docusaurus patterns.
