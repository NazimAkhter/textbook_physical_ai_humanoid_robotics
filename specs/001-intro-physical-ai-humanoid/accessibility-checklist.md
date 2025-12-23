# WCAG 2.1 AA Accessibility Validation Checklist

**Purpose**: Comprehensive checklist for validating introduction.md meets WCAG 2.1 Level AA standards
**Date**: 2025-12-23
**Feature**: 001-intro-physical-ai-humanoid
**Standard**: Web Content Accessibility Guidelines (WCAG) 2.1 Level AA

---

## Perceivable

Content must be presented in ways all users can perceive.

### 1.1 Text Alternatives

- [ ] **Alt Text for Diagrams**: All 4 SVG diagrams have descriptive alt text (50-100 words each)
  - [ ] comparative-diagram.svg alt text describes content, not just title
  - [ ] physical-ai-concept-map.svg alt text explains relationships
  - [ ] architecture-diagram.svg alt text describes data flow
  - [ ] learning-path-flowchart.svg alt text explains progression
- [ ] **Decorative Images**: Any purely decorative images have empty alt text (`alt=""`)
- [ ] **Complex Images**: Diagrams with complex information have extended descriptions if needed

### 1.3 Adaptable

- [ ] **Semantic HTML**: Proper heading hierarchy (h1 → h2 → h3, no skipped levels)
  - [ ] Only one h1 (page title)
  - [ ] Section headings use h2
  - [ ] Subsections use h3
  - [ ] No heading level skips
- [ ] **Lists**: Related items use proper list markup (`<ul>`, `<ol>`)
- [ ] **Callout Boxes**: Docusaurus admonitions use semantic markup (not just visual styling)
- [ ] **Reading Order**: Content makes sense when read linearly without CSS
- [ ] **Meaningful Sequence**: Information flows logically from top to bottom

### 1.4 Distinguishable

- [ ] **Color Contrast - Text**: All text has minimum 4.5:1 contrast ratio against background
  - [ ] Body text (16px or larger): 4.5:1 minimum
  - [ ] Heading text: 4.5:1 minimum (or 3:1 if 18pt+/24px+ bold)
  - [ ] Link text: 4.5:1 minimum
- [ ] **Color Contrast - Graphics**: Diagrams have 3:1 contrast minimum for meaningful elements
  - [ ] Diagram borders and shapes: 3:1 minimum
  - [ ] Diagram text: 4.5:1 minimum
- [ ] **Color Not Sole Indicator**: Information not conveyed by color alone
  - [ ] Links have underline or other non-color indicator
  - [ ] Diagram elements have labels/patterns, not just color coding
- [ ] **Resize Text**: Content readable and functional when text size increased to 200%
- [ ] **Images of Text**: No images of text (except diagrams where necessary)
  - [ ] All diagrams use SVG text (not rasterized text)
- [ ] **Reflow**: Content reflows to single column on mobile without horizontal scroll

---

## Operable

User interface components and navigation must be operable.

### 2.1 Keyboard Accessible

- [ ] **Keyboard Navigation**: All interactive elements accessible via keyboard
  - [ ] Tab order follows logical reading order
  - [ ] No keyboard traps (can tab in and out of all elements)
  - [ ] Skip navigation link present (Docusaurus default)
- [ ] **Focus Order**: Tab order matches visual order
- [ ] **Focus Visible**: Keyboard focus indicator visible at all times
  - [ ] Links show focus outline
  - [ ] Interactive callouts show focus
  - [ ] Search/navigation elements show focus

### 2.4 Navigable

- [ ] **Page Title**: Descriptive page title present (from frontmatter)
- [ ] **Focus Order**: Logical tab order through content
- [ ] **Link Purpose**: Link text describes destination (no "click here")
  - [ ] Internal links to modules clearly labeled
  - [ ] External reference links describe target
- [ ] **Multiple Ways**: Content accessible via sidebar, search, and direct URL (Docusaurus default)
- [ ] **Headings and Labels**: Headings are descriptive and accurate
- [ ] **Visible Focus**: Focus indicator has 3:1 contrast minimum

### 2.5 Input Modalities

- [ ] **Pointer Gestures**: No complex gestures required (not applicable to text content)
- [ ] **Pointer Cancellation**: Clicking doesn't trigger on down-event (not applicable to text content)
- [ ] **Label in Name**: Visible labels match accessible names (Docusaurus handles this)
- [ ] **Motion Actuation**: No motion-based activation required

---

## Understandable

Information and user interface operation must be understandable.

### 3.1 Readable

- [ ] **Language of Page**: HTML lang attribute set (Docusaurus default: `lang="en"`)
- [ ] **Language of Parts**: Any non-English terms marked with appropriate lang attribute
  - [ ] Technical terms defined in context or glossary
  - [ ] Acronyms spelled out on first use
- [ ] **Reading Level**: Content appropriate for target audience (AI/robotics engineers)
  - [ ] Technical terms explained in Prerequisite Refresher callouts
  - [ ] Jargon minimized or defined

### 3.2 Predictable

- [ ] **On Focus**: No automatic context changes when element receives focus
- [ ] **On Input**: No automatic context changes when input value changes (not applicable to text content)
- [ ] **Consistent Navigation**: Sidebar navigation consistent (Docusaurus handles this)
- [ ] **Consistent Identification**: Callout boxes consistently formatted
  - [ ] "Prerequisite Refresher" always uses same admonition type (:::note or :::info)
  - [ ] "Deep Dive" always uses same admonition type (:::tip)

### 3.3 Input Assistance

- [ ] **Error Identification**: If errors occur (e.g., broken links), clearly described (automated testing)
- [ ] **Labels or Instructions**: All form elements labeled (not applicable to text content)
- [ ] **Error Suggestion**: Error correction suggestions provided (not applicable to text content)

---

## Robust

Content must be robust enough to work with assistive technologies.

### 4.1 Compatible

- [ ] **Valid HTML**: Docusaurus generates valid HTML (automated via build)
  - [ ] No duplicate IDs
  - [ ] Proper nesting of elements
  - [ ] Closing tags present
- [ ] **Name, Role, Value**: All UI components have accessible name and role
  - [ ] Links have accessible names
  - [ ] Buttons have accessible names (not applicable to text content)
  - [ ] Custom components have ARIA labels if needed
- [ ] **Status Messages**: Status changes announced to screen readers (Docusaurus handles search/navigation)

---

## Docusaurus-Specific Checks

### Theme and Layout

- [ ] **Dark Mode Support**: Content readable in both light and dark themes
  - [ ] Test all diagrams in dark mode
  - [ ] Verify text contrast in dark mode
- [ ] **Mobile Responsive**: Content works on mobile devices (320px width minimum)
  - [ ] Diagrams scale appropriately
  - [ ] Text reflows without horizontal scroll
  - [ ] Navigation accessible on mobile
- [ ] **Font Sizing**: Base font size minimum 16px (Docusaurus default)

### Navigation

- [ ] **Sidebar**: Introduction appears in correct position (sidebar_position: 1)
- [ ] **Breadcrumbs**: Breadcrumb trail accessible via keyboard
- [ ] **Search**: Content indexed for site search (Docusaurus automatic)
- [ ] **Table of Contents**: Right sidebar TOC keyboard accessible

### Callout Boxes (Admonitions)

- [ ] **Semantic Markup**: Admonitions use proper HTML structure
- [ ] **Icon Accessibility**: Icons have appropriate ARIA labels or are aria-hidden
- [ ] **Color Independence**: Callout type distinguishable without color (icons + labels)
- [ ] **Keyboard Access**: Collapsible admonitions (if used) keyboard accessible

---

## Testing Tools and Methods

### Automated Testing

- [ ] **Lighthouse Audit**: Run Lighthouse accessibility audit (target: 90+ score)
  - Command: Chrome DevTools → Lighthouse → Accessibility
- [ ] **axe DevTools**: Run axe browser extension scan (target: 0 violations)
  - Extension: https://www.deque.com/axe/devtools/
- [ ] **WAVE**: Run WAVE Web Accessibility Evaluation Tool
  - Extension: https://wave.webaim.org/extension/
- [ ] **HTML Validator**: Validate HTML markup
  - Tool: https://validator.w3.org/

### Manual Testing

- [ ] **Keyboard Navigation**: Tab through entire page without mouse
  - [ ] Verify all links accessible
  - [ ] Verify focus indicators visible
  - [ ] Verify no keyboard traps
- [ ] **Screen Reader Testing**: Test with NVDA (Windows) or VoiceOver (Mac)
  - [ ] All headings announced correctly
  - [ ] All diagrams have alt text read aloud
  - [ ] Callout boxes announced with appropriate context
  - [ ] Links describe destination
- [ ] **Zoom Testing**: Test at 200% zoom level
  - [ ] Content reflows appropriately
  - [ ] No text truncation
  - [ ] No horizontal scroll
- [ ] **Contrast Testing**: Verify all color combinations
  - Tool: WebAIM Contrast Checker (https://webaim.org/resources/contrastchecker/)
  - [ ] Body text vs background
  - [ ] Link text vs background
  - [ ] Diagram colors vs background

### User Testing (Optional)

- [ ] **Real User Testing**: Have users with disabilities test the content
- [ ] **Diverse Browsers**: Test in Chrome, Firefox, Safari, Edge
- [ ] **Diverse Devices**: Test on desktop, tablet, mobile

---

## Common Pitfalls to Avoid

### Content Issues

- ❌ Using "click here" or "read more" as link text
- ❌ Relying on color alone to convey information
- ❌ Images of text instead of actual text
- ❌ Poor heading hierarchy (skipping levels)
- ❌ Empty headings or links

### Diagram Issues

- ❌ Missing alt text on diagrams
- ❌ Alt text that just says "diagram" or "image"
- ❌ Low contrast colors in diagrams
- ❌ Text in diagrams that doesn't scale
- ❌ Diagrams that don't work in dark mode

### Navigation Issues

- ❌ Links that open new windows without warning
- ❌ Keyboard focus not visible
- ❌ Illogical tab order
- ❌ Navigation that skips content regions

---

## Success Criteria

**To pass WCAG 2.1 Level AA, the introduction.md page must**:

1. ✅ Score 90+ on Lighthouse accessibility audit
2. ✅ Have 0 critical or serious issues in axe DevTools
3. ✅ Pass keyboard navigation test (all elements accessible)
4. ✅ Pass screen reader test (all content announced correctly)
5. ✅ Pass contrast test (all text 4.5:1, graphics 3:1)
6. ✅ Pass zoom test (readable at 200%)
7. ✅ Have descriptive alt text for all diagrams
8. ✅ Use proper semantic HTML throughout

---

## Remediation Workflow

**If accessibility issues found**:

1. **Identify**: Document specific WCAG criterion violated
2. **Prioritize**: Address Level A issues first, then AA
3. **Fix**: Implement correction (update content, add alt text, fix contrast)
4. **Test**: Rerun automated and manual tests
5. **Verify**: Confirm issue resolved
6. **Document**: Note change in commit message

---

## References

- WCAG 2.1 Guidelines: https://www.w3.org/WAI/WCAG21/quickref/
- Docusaurus Accessibility: https://docusaurus.io/docs/accessibility
- WebAIM Resources: https://webaim.org/
- A11y Project Checklist: https://www.a11yproject.com/checklist/

---

**Status**: Checklist ready for use in Phase 6 (Polish & Validation)
**Usage**: Run this checklist after all content and diagrams are complete
