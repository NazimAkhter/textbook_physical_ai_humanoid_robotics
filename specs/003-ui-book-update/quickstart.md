# Quickstart: UI Update Implementation

**Feature**: `003-ui-book-update`
**Date**: 2024-12-15

## Prerequisites

- Node.js 18+ installed
- Git repository cloned
- On branch `003-ui-book-update`

## Quick Verification

```bash
# Verify branch
git branch --show-current
# Expected: 003-ui-book-update

# Install dependencies (if not done)
cd frontend
npm install

# Start development server
npm run start
# Visit http://localhost:3000
```

## Implementation Order

### Phase 1: Theme Colors (US2)

1. **Update `src/css/custom.css`**:
   - Replace default green primary with indigo/violet
   - Add dark mode color overrides
   - Verify colors with build

```bash
# After changes
npm run build
# Should complete without errors
```

### Phase 2: Configuration Updates (US3)

2. **Update `docusaurus.config.js`**:
   - Change `title` to "Physical AI Book"
   - Update `tagline` to new messaging
   - Modify `navbar.items` to remove Tutorial link
   - Update `footer` to remove placeholder content

```bash
# After changes
npm run start
# Verify navbar and footer render correctly
```

### Phase 3: Module Cards Component (US1 - MVP)

3. **Create `src/components/ModuleCards/`**:
   - Create `index.js` with card component
   - Create `styles.module.css` with responsive grid
   - Define module data array

4. **Update `src/pages/index.js`**:
   - Import ModuleCards component
   - Replace HomepageFeatures with ModuleCards
   - Update hero CTA link

```bash
# After changes
npm run start
# Verify:
# - Hero section displays new branding
# - Module cards appear in grid
# - Clicking cards navigates to modules
# - Responsive layout at different widths
```

### Phase 4: Link Validation (US4)

5. **Audit and fix all links**:
   - Run full build to catch broken links
   - Verify CTA button links to Module 1
   - Verify navbar links work
   - Verify footer links work

```bash
# Final validation
npm run build
# Must complete with 0 broken link warnings

npm run serve
# Manual test all navigation
```

## Validation Commands

```bash
# Full build (catches broken links)
cd frontend && npm run build

# Development server
npm run start

# Production preview
npm run serve
```

## Common Issues

### Issue: "Broken link" error during build

**Cause**: Internal link points to non-existent page.
**Fix**: Check path in component/config matches actual file path in `docs/`.

### Issue: Colors don't update

**Cause**: Browser caching CSS.
**Fix**: Hard refresh (Ctrl+Shift+R) or clear cache.

### Issue: Module cards not responsive

**Cause**: Missing media queries in CSS.
**Fix**: Verify `styles.module.css` has `@media` rules for breakpoints.

## File Checklist

Files to **CREATE**:
- [ ] `src/components/ModuleCards/index.js`
- [ ] `src/components/ModuleCards/styles.module.css`

Files to **MODIFY**:
- [ ] `src/css/custom.css` (theme colors)
- [ ] `src/pages/index.js` (hero + module cards)
- [ ] `src/pages/index.module.css` (hero styles)
- [ ] `docusaurus.config.js` (title, navbar, footer)

Files to **DELETE** (optional cleanup):
- [ ] `src/components/HomepageFeatures/` (replaced by ModuleCards)

## Success Criteria Verification

| Criterion | How to Verify |
|-----------|---------------|
| SC-001: Page loads < 3s | Browser DevTools → Network tab |
| SC-002: No broken links | `npm run build` succeeds |
| SC-003: Responsive layout | Browser DevTools → Device toolbar |
| SC-004: Theme colors | Visual inspection |
| SC-005: Navbar branding | Visual inspection |
| SC-006: Dark mode contrast | Toggle dark mode, use contrast checker |
| SC-007: Clean build | `npm run build` with 0 warnings |
