# Contract: Theme Color Variables

**Feature**: 006-ui-theme-update
**Contract Type**: CSS Variable Definitions
**Scope**: Global theme colors for light and dark modes

## Purpose

Define the complete set of CSS custom properties that must be implemented in `frontend/src/css/custom.css` to provide a unified teal/cyan color scheme across the Docusaurus site.

---

## Contract Specification

### Light Mode Colors (`:root` selector)

**MUST Define**:

```css
:root {
  /* Primary Color Palette */
  --ifm-color-primary: #115e59;
  --ifm-color-primary-dark: #0d4a46;
  --ifm-color-primary-darker: #0c4543;
  --ifm-color-primary-darkest: #0a3938;
  --ifm-color-primary-light: #14736d;
  --ifm-color-primary-lighter: #168078;
  --ifm-color-primary-lightest: #1a9891;

  /* Link Colors */
  --ifm-link-color: #115e59;
  --ifm-link-hover-color: #0d4a46;
  --ifm-link-decoration: none;

  /* Code Block Colors */
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(17, 94, 89, 0.1);

  /* Footer Colors */
  --ifm-footer-background-color: #f8f9fa;
  --ifm-footer-color: #1b1b1d;
  --ifm-footer-link-color: #115e59;
  --ifm-footer-link-hover-color: #0d4a46;
  --ifm-footer-title-color: #1b1b1d;
}
```

### Dark Mode Colors (`[data-theme='dark']` selector)

**MUST Define**:

```css
[data-theme='dark'] {
  /* Primary Color Palette */
  --ifm-color-primary: #99f6e4;
  --ifm-color-primary-dark: #5eead4;
  --ifm-color-primary-darker: #2dd4bf;
  --ifm-color-primary-darkest: #14b8a6;
  --ifm-color-primary-light: #ccfbf1;
  --ifm-color-primary-lighter: #f0fdfa;
  --ifm-color-primary-lightest: #ffffff;

  /* Link Colors */
  --ifm-link-color: #99f6e4;
  --ifm-link-hover-color: #5eead4;

  /* Code Block Colors */
  --docusaurus-highlighted-code-line-bg: rgba(153, 246, 228, 0.1);

  /* Footer Colors */
  --ifm-footer-background-color: #1b1b1d;
  --ifm-footer-color: #e3e3e3;
  --ifm-footer-link-color: #99f6e4;
  --ifm-footer-link-hover-color: #5eead4;
  --ifm-footer-title-color: #ffffff;
}
```

---

## Acceptance Criteria

**For Light Mode**:
- [ ] Primary color is exactly #115e59
- [ ] All color shades (dark, darker, darkest, light, lighter, lightest) are defined
- [ ] Link color matches primary color
- [ ] Link hover color is visually darker than link color
- [ ] Footer background is light gray (#f8f9fa)
- [ ] Footer links use primary teal color
- [ ] All text colors meet WCAG AA contrast (4.5:1 minimum on respective backgrounds)

**For Dark Mode**:
- [ ] Primary color is exactly #99f6e4
- [ ] All color shades are defined with appropriate progression
- [ ] Link color matches primary color
- [ ] Link hover color is visually darker (less bright) than link color
- [ ] Footer background is dark gray (#1b1b1d)
- [ ] Footer links use primary cyan color
- [ ] All text colors meet WCAG AA contrast (4.5:1 minimum on respective backgrounds)

**For Both Modes**:
- [ ] CSS variables are defined at document root level (`:root` or `[data-theme='dark']`)
- [ ] Variable names follow Infima CSS framework conventions (`--ifm-` prefix)
- [ ] No hardcoded colors in component styles (all use CSS variables)
- [ ] Color transitions smooth when toggling dark mode (<200ms)

---

## Validation Tests

### Test 1: Color Values

**Input**: Load `frontend/src/css/custom.css`

**Expected Output**:
```
Light mode primary: #115e59 ✅
Dark mode primary: #99f6e4 ✅
All shades defined: 7 per mode ✅
```

### Test 2: Contrast Ratios

**Input**: Check contrast between:
- Light mode: #115e59 text on #FFFFFF background
- Dark mode: #99f6e4 text on #1b1b1d background

**Expected Output**:
```
Light mode contrast: ≥4.5:1 ✅
Dark mode contrast: ≥4.5:1 ✅
```

### Test 3: Visual Consistency

**Input**: Navigate through:
- Homepage
- Module 1 documentation
- Any content page

**Expected Output**: All links, buttons, highlights use the same teal/cyan palette across all pages ✅

### Test 4: Mode Toggle

**Input**: Click dark mode toggle button

**Expected Output**:
- All colors update within 200ms ✅
- No flash of unstyled content ✅
- Footer colors change to match mode ✅

---

## Error Handling

**Invalid Color Value**:
- Browser will ignore the invalid variable
- Fallback to Infima default blue (#2e8555)
- **Prevention**: Validate hex codes during implementation

**Missing Variable**:
- Component will use Infima default
- May cause inconsistent branding
- **Prevention**: Define all variables in contract before testing

**Incorrect Selector**:
- Dark mode colors won't apply
- Light mode colors used in both modes
- **Prevention**: Use `[data-theme='dark']` not `[data-theme='night']`

---

## Dependencies

**Upstream**:
- Docusaurus 3.x Infima CSS framework (provides base variables)
- Browser CSS custom property support (all modern browsers)

**Downstream**:
- All React components that use CSS modules
- Hero section component (requires these variables for buttons)
- Footer component (requires footer-specific variables)
- Navbar component (inherits primary color for active links)

---

## Notes

- These variables override Docusaurus defaults but maintain Infima naming conventions
- Shades (dark, darker, darkest, etc.) are used by Infima for button hover states, shadows, and borders
- Code block highlighting uses semi-transparent versions of primary color for consistency
- Footer variables are custom additions (not standard Infima) to meet FR-015 and FR-016
