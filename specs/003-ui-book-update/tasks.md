# Tasks: UI Update for Physical AI Book Website

**Input**: Design documents from `/specs/003-ui-book-update/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅, quickstart.md ✅
**Branch**: `003-ui-book-update`

**Tests**: Manual testing only (no automated tests requested in spec)

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)

## Path Conventions

- **Project structure**: `frontend/` contains Docusaurus site
- **Source code**: `frontend/src/`
- **Components**: `frontend/src/components/`
- **Styles**: `frontend/src/css/`
- **Pages**: `frontend/src/pages/`
- **Config**: `frontend/docusaurus.config.js`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify development environment is ready

- [x] T001 Verify branch is `003-ui-book-update` and working directory is clean
- [x] T002 Run `npm install` in `frontend/` to ensure dependencies are current
- [x] T003 Run `npm run start` to verify dev server starts without errors

---

## Phase 2: Foundational (Theme Configuration)

**Purpose**: Core theme changes that affect ALL user stories - MUST complete before user story work

**⚠️ CRITICAL**: No user story work should begin until theme colors are in place

- [x] T004 [US2] Update `frontend/src/css/custom.css` with indigo/violet light mode palette
- [x] T005 [US2] Update `frontend/src/css/custom.css` with dark mode colors using `[data-theme='dark']`
- [x] T006 Run `npm run build` to verify CSS compiles without errors

**Checkpoint**: Theme colors ready - component work can now begin

---

## Phase 3: User Story 2 - Updated Visual Theme (Priority: P2)

**Goal**: Site displays consistent indigo/violet color theme across all pages with proper dark mode support

**Independent Test**: Toggle dark mode and verify colors change appropriately with good contrast

### Implementation for User Story 2

- [x] T007 [P] [US2] Verify hero section uses new theme colors (inspect in browser)
- [x] T008 [P] [US2] Verify navbar uses new theme colors
- [x] T009 [US2] Test dark mode toggle - verify colors and contrast

**Checkpoint**: Theme is visually complete and verified

---

## Phase 4: User Story 3 - Professional Navigation Bar (Priority: P3)

**Goal**: Navigation bar displays "Physical AI Book" branding with correct links

**Independent Test**: All navbar items resolve correctly, mobile hamburger menu works

### Implementation for User Story 3

- [x] T010 [US3] Update `frontend/docusaurus.config.js` - change `title` to "Physical AI Book"
- [x] T011 [US3] Update `frontend/docusaurus.config.js` - change `tagline` to "Master humanoid robotics with ROS 2, digital twins, and Vision-Language-Action systems"
- [x] T012 [US3] Update `frontend/docusaurus.config.js` - update `navbar.title` to "Physical AI Book"
- [x] T013 [US3] Update `frontend/docusaurus.config.js` - change navbar `Tutorial` item label to "Modules"
- [x] T014 [US3] Update `frontend/docusaurus.config.js` - remove Blog link from navbar items
- [x] T015 [US3] Update `frontend/docusaurus.config.js` - update GitHub href to project repository
- [x] T016 [US3] Verify mobile hamburger menu displays all navigation items

**Checkpoint**: Navbar branding and links are correct

---

## Phase 5: User Story 1 - Homepage Module Navigation (Priority: P1) 🎯 MVP

**Goal**: Homepage displays module cards in responsive grid; clicking navigates to correct module

**Independent Test**: User can land on homepage, see module cards, click any card, and navigate to correct module content

### Create ModuleCards Component

- [x] T017 [US1] Create directory `frontend/src/components/ModuleCards/`
- [x] T018 [US1] Create `frontend/src/components/ModuleCards/index.js` with module data array and card component
- [x] T019 [US1] Create `frontend/src/components/ModuleCards/styles.module.css` with responsive grid layout
- [x] T020 [US1] Verify ModuleCards component exports correctly

### Update Homepage

- [x] T021 [US1] Update `frontend/src/pages/index.js` - import ModuleCards component
- [x] T022 [US1] Update `frontend/src/pages/index.js` - replace HomepageFeatures with ModuleCards in main section
- [x] T023 [US1] Update `frontend/src/pages/index.js` - update hero CTA button text to "Start Learning"
- [x] T024 [US1] Update `frontend/src/pages/index.js` - update hero CTA link to `/docs/category/module-1-ros-2-nervous-system`
- [x] T025 [US1] Update `frontend/src/pages/index.js` - update Layout description meta tag

### Hero Section Styling

- [x] T026 [P] [US1] Update `frontend/src/pages/index.module.css` - enhance hero banner styling if needed

### Verify Module Cards

- [x] T027 [US1] Test Module 1 card links to `/docs/category/module-1-ros-2-nervous-system`
- [x] T028 [US1] Test Module 2 card links to `/docs/category/module-2-digital-twins`
- [x] T029 [US1] Verify Modules 3-5 display "Coming Soon" badge and are not clickable
- [x] T030 [US1] Test responsive layout at 375px (mobile - 1 column)
- [x] T031 [US1] Test responsive layout at 768px (tablet - 2 columns)
- [x] T032 [US1] Test responsive layout at 1440px (desktop - 3 columns)

**Checkpoint**: Module cards render correctly and all links work

---

## Phase 6: User Story 4 - Broken Link Resolution (Priority: P4)

**Goal**: All internal navigation links resolve correctly (0 broken links)

**Independent Test**: `npm run build` completes with no broken link warnings

### Footer Configuration

- [x] T033 [US4] Update `frontend/docusaurus.config.js` - simplify footer `links` to Learn and More sections only
- [x] T034 [US4] Update `frontend/docusaurus.config.js` - remove Community section from footer
- [x] T035 [US4] Update `frontend/docusaurus.config.js` - update footer Learn section to link to `/docs/intro`
- [x] T036 [US4] Update `frontend/docusaurus.config.js` - update footer More section GitHub href
- [x] T037 [US4] Update `frontend/docusaurus.config.js` - update footer `copyright` to "Physical AI Book"

### Link Audit

- [x] T038 [US4] Run `npm run build` and verify 0 broken link warnings
- [x] T039 [US4] Manually verify hero CTA button navigates to Module 1
- [x] T040 [US4] Manually verify all navbar links resolve
- [x] T041 [US4] Manually verify all footer links resolve

**Checkpoint**: All links validated, build passes

---

## Phase 7: Cleanup & Cross-Cutting Concerns

**Purpose**: Final cleanup and validation

### Cleanup

- [x] T042 [P] Remove or archive `frontend/src/components/HomepageFeatures/` (optional - can keep for reference)
- [x] T043 [P] Remove default tutorial folders from `frontend/docs/` if not needed (tutorial-basics, tutorial-extras)

### Final Validation

- [x] T044 Run full `npm run build` - must complete with exit code 0
- [x] T045 Run `npm run serve` - test production build locally
- [x] T046 Test dark mode toggle throughout site
- [x] T047 Verify color contrast meets WCAG AA (4.5:1 for text)
- [x] T048 Test keyboard navigation (Tab through all interactive elements)
- [x] T049 Run quickstart.md validation checklist

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - provides theme colors for all components
- **User Story 2 (Phase 3)**: Depends on Foundational - theme verification
- **User Story 3 (Phase 4)**: Depends on Setup - can run parallel to Phase 3
- **User Story 1 (Phase 5)**: Depends on Foundational - needs theme colors for cards
- **User Story 4 (Phase 6)**: Depends on Phases 3-5 - audits links created in those phases
- **Cleanup (Phase 7)**: Depends on all user stories complete

### Within Each Phase

- Tasks marked [P] can run in parallel (different files)
- Complete config changes before verification tasks
- Build validation after each significant change

### Recommended Execution Order

```
Phase 1 (Setup)
    ↓
Phase 2 (Theme) ─────────────────┐
    ↓                            │
Phase 3 (US2: Theme verify) ←────┘
    ↓
Phase 4 (US3: Navbar) ──┐
    ↓                   │ (can parallel)
Phase 5 (US1: Cards) ←──┘
    ↓
Phase 6 (US4: Links)
    ↓
Phase 7 (Cleanup)
```

---

## Implementation Strategy

### MVP First (User Story 1)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (theme colors)
3. Complete Phase 5: User Story 1 (module cards)
4. **STOP and VALIDATE**: Homepage works with navigation
5. Continue with remaining stories

### Full Implementation

1. Setup → Foundational → Theme verify
2. Navbar updates (config changes)
3. Module cards component (new files)
4. Footer and link audit (config cleanup)
5. Final validation

---

## Notes

- All file paths are relative to repository root
- `[P]` indicates tasks that can safely run in parallel
- Config changes (`docusaurus.config.js`) should be grouped to minimize rebuilds
- Test in both light and dark mode after each visual change
- Commit after each phase or logical group of tasks
- The existing `HomepageFeatures` component can be kept as reference or deleted

---

## Task Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| 1 | T001-T003 | Setup verification |
| 2 | T004-T006 | Theme colors (foundational) |
| 3 | T007-T009 | US2: Theme verification |
| 4 | T010-T016 | US3: Navbar configuration |
| 5 | T017-T032 | US1: Module cards (MVP) |
| 6 | T033-T041 | US4: Footer + link audit |
| 7 | T042-T049 | Cleanup + validation |

**Total**: 49 tasks across 7 phases
