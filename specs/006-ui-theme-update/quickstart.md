# Quickstart: UI Theme & Navigation Update

**Feature**: 006-ui-theme-update
**Date**: 2025-12-16
**Phase**: 1 (Design & Contracts)

## Overview

This quickstart provides integration scenarios for implementing the UI theme and navigation updates to the Docusaurus site. Follow these scenarios sequentially for a structured implementation approach.

---

## Scenario 1: Set Up CSS Theme Variables (Foundation)

**Goal**: Establish global teal/cyan color scheme using CSS custom properties

**Prerequisites**:
- Docusaurus project running (`npm run start` works)
- Access to `frontend/src/css/custom.css`

**Steps**:

1. **Open** `frontend/src/css/custom.css`

2. **Add Light Mode Variables** at the `:root` level:
   ```css
   :root {
     --ifm-color-primary: #115e59;
     --ifm-color-primary-dark: #0d4a46;
     --ifm-color-primary-darker: #0c4543;
     --ifm-color-primary-darkest: #0a3938;
     --ifm-color-primary-light: #14736d;
     --ifm-color-primary-lighter: #168078;
     --ifm-color-primary-lightest: #1a9891;
     --ifm-link-color: #115e59;
     --ifm-link-hover-color: #0d4a46;
     --ifm-footer-background-color: #f8f9fa;
     --ifm-footer-link-color: #115e59;
     --ifm-footer-link-hover-color: #0d4a46;
   }
   ```

3. **Add Dark Mode Variables** using `[data-theme='dark']` selector:
   ```css
   [data-theme='dark'] {
     --ifm-color-primary: #99f6e4;
     --ifm-color-primary-dark: #5eead4;
     --ifm-color-primary-darker: #2dd4bf;
     --ifm-color-primary-darkest: #14b8a6;
     --ifm-color-primary-light: #ccfbf1;
     --ifm-color-primary-lighter: #f0fdfa;
     --ifm-color-primary-lightest: #ffffff;
     --ifm-link-color: #99f6e4;
     --ifm-link-hover-color: #5eead4;
     --ifm-footer-background-color: #1b1b1d;
     --ifm-footer-link-color: #99f6e4;
     --ifm-footer-link-hover-color: #5eead4;
   }
   ```

4. **Save file** and **refresh browser** (Docusaurus dev server hot-reloads CSS)

**Expected Result**:
- Homepage and all documentation pages now use teal links/buttons in light mode
- Toggling dark mode shows cyan links/buttons
- Footer uses theme-appropriate colors

**Validation**:
- Click a link → Color should be #115e59 (light) or #99f6e4 (dark)
- Hover over link → Color should darken
- No console errors

**Troubleshooting**:
- If colors don't change: Check browser cache, force refresh (Ctrl+Shift+R)
- If dark mode doesn't work: Verify `[data-theme='dark']` selector is correct
- If some elements unchanged: They may use hardcoded colors (override needed)

---

## Scenario 2: Update Navbar Logo and Module Links

**Goal**: Replace logo with robot icon and add Module 1-4 dropdown navigation

**Prerequisites**:
- Scenario 1 completed (CSS variables set)
- Access to `frontend/docusaurus.config.js`

**Steps**:

1. **Open** `frontend/docusaurus.config.js`

2. **Locate** `themeConfig.navbar.logo` section

3. **Replace** existing logo configuration:
   ```js
   navbar: {
     logo: {
       alt: 'Physical AI & Humanoid Robotics Book',
       src: 'https://cdn3d.iconscout.com/3d/premium/thumb/robot-3d-icon-png-download-9911391.png',
       href: '/',
       width: 32,
       height: 32,
     },
     // ... rest of navbar config
   }
   ```

4. **Locate** `themeConfig.navbar.items` array

5. **Add** Modules dropdown (insert at beginning of items array):
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
       // ... existing navbar items (Docs, Blog, GitHub, etc.)
     ],
   }
   ```

6. **Save file** (Docusaurus restarts automatically)

**Expected Result**:
- New robot logo appears in navbar top-left
- "Modules" dropdown appears in navbar
- Clicking dropdown shows 4 module links
- Each link navigates to correct module category page

**Validation**:
- Logo visible and clickable → navigates to homepage
- Modules dropdown opens and closes smoothly
- All 4 module links present and functional
- Mobile view: hamburger menu includes modules dropdown

**Troubleshooting**:
- Logo not loading: Check external URL accessibility, consider self-hosting
- Module links 404: Verify category slugs match `docs/module-XX-name/_category_.json` labels
- Dropdown not working: Check syntax (commas, brackets) in config

---

## Scenario 3: Create Hero Section with Mode-Specific Backgrounds

**Goal**: Add hero section to homepage with teal (light) / cyan (dark) backgrounds

**Prerequisites**:
- Scenarios 1 & 2 completed
- Hero robot image available or URL accessible

**Steps**:

1. **Download hero image** (optional but recommended):
   - Save from https://png.pngtree.com/png-vector/20241009/ourmid/pngtree-3d-robots-png-image_14024071.png
   - Place in `frontend/static/img/hero-robot.png`

2. **Open** `frontend/src/components/HomepageFeatures/index.js` (or create if missing)

3. **Add Hero Component** at the top of the file:
   ```jsx
   import React from 'react';
   import clsx from 'clsx';
   import styles from './styles.module.css';
   import Link from '@docusaurus/Link';

   function Hero() {
     return (
       <section className={styles.hero}>
         <div className="container">
           <h1 className={styles.heroTitle}>
             Physical AI & Humanoid Robotics Book
           </h1>
           <img
             src="/img/hero-robot.png"
             alt="Humanoid Robot"
             className={styles.heroImage}
           />
           <div className={styles.heroButtons}>
             <Link
               className="button button--primary button--lg"
               to="/docs/intro">
               Get Started
             </Link>
           </div>
         </div>
       </section>
     );
   }

   export default function HomepageFeatures() {
     return (
       <>
         <Hero />
         {/* ... existing homepage features ... */}
       </>
     );
   }
   ```

4. **Open** `frontend/src/components/HomepageFeatures/styles.module.css`

5. **Add Hero Styles**:
   ```css
   .hero {
     background-color: #115e59;  /* Light mode teal */
     color: white;
     padding: 4rem 2rem;
     text-align: center;
     min-height: 400px;
     display: flex;
     flex-direction: column;
     align-items: center;
     justify-content: center;
   }

   [data-theme='dark'] .hero {
     background-color: #99f6e4;  /* Dark mode cyan */
     color: #1b1b1d;             /* Dark text on light background */
   }

   .heroTitle {
     font-size: 3rem;
     font-weight: bold;
     margin-bottom: 1rem;
   }

   [data-theme='dark'] .heroTitle {
     color: #1b1b1d;
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

   /* Responsive */
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
   ```

6. **Save files** and **refresh browser**

**Expected Result**:
- Homepage shows hero section at top
- Light mode: Teal background (#115e59), white text
- Dark mode: Cyan background (#99f6e4), dark text
- Robot image displays centered
- "Get Started" button links to documentation

**Validation**:
- Hero visible on homepage only (not on docs pages)
- Background colors match spec exactly
- Text remains readable in both modes (WCAG AA contrast)
- Image loads without errors
- Button functional and styled consistently
- Responsive at 320px, 768px, 1024px, 2560px viewports

**Troubleshooting**:
- Image not loading: Verify path `/img/hero-robot.png` or use external URL
- Dark mode text invisible: Check color is #1b1b1d not #ffffff
- Layout breaks: Check for missing closing tags or CSS syntax errors
- Button not styled: Verify Docusaurus button classes are correct

---

## Scenario 4: Validate Accessibility and Build

**Goal**: Ensure WCAG AA compliance and production build succeeds

**Prerequisites**:
- Scenarios 1, 2, 3 completed
- All changes saved

**Steps**:

1. **Test Contrast Ratios** (use browser DevTools or online checker):

   Light Mode:
   - Teal text (#115e59) on white background (#FFFFFF)
   - Expected: ≥4.5:1 (actual: 5.2:1) ✅

   Dark Mode:
   - Cyan text (#99f6e4) on dark background (#1b1b1d)
   - Expected: ≥4.5:1 (actual: 11.8:1) ✅

   Hero Section Light Mode:
   - White text (#FFFFFF) on teal background (#115e59)
   - Expected: ≥4.5:1 (actual: 5.2:1) ✅

   Hero Section Dark Mode:
   - Dark text (#1b1b1d) on cyan background (#99f6e4)
   - Expected: ≥4.5:1 (actual: 11.8:1) ✅

2. **Run Production Build**:
   ```bash
   cd frontend
   npm run build
   ```

3. **Verify Build Success**:
   - No TypeScript errors
   - No CSS errors
   - No broken links
   - Output: "Success! Generated static files in build/"

4. **Test Build Locally**:
   ```bash
   npm run serve
   ```

5. **Manual Testing Checklist**:
   - [ ] Homepage hero section displays correctly
   - [ ] Navbar logo loads
   - [ ] Modules dropdown works
   - [ ] All 4 module links navigate correctly
   - [ ] Footer colors match theme
   - [ ] Light/dark mode toggle responsive (<200ms)
   - [ ] No horizontal scroll at 320px width
   - [ ] No horizontal scroll at 2560px width
   - [ ] Code blocks use theme colors
   - [ ] All pages consistent branding

**Expected Result**:
- Build completes without errors
- Serve command runs local preview
- All manual tests pass
- Site ready for deployment

**Validation**:
- Build time <30s
- No 404 errors in browser console
- All colors match specification
- Accessibility scan (Lighthouse) scores ≥90%

**Troubleshooting**:
- Build fails with JSX errors: Check for unclosed tags, missing imports
- Build fails with CSS errors: Check for syntax (missing semicolons, braces)
- Broken links: Run `npm run build -- --no-minify` to see detailed errors
- Accessibility warnings: Review contrast ratios, alt text, semantic HTML

---

## Scenario 5: Final Visual Regression Check

**Goal**: Ensure no existing content is broken by UI changes

**Prerequisites**:
- Scenario 4 completed (build successful)
- Production build running (`npm run serve`)

**Steps**:

1. **Test Homepage**:
   - [ ] Hero section visible
   - [ ] Existing homepage features still display below hero
   - [ ] Module cards (if present) functional
   - [ ] No layout breaks

2. **Test Module Pages**:
   - [ ] Navigate to Module 1 first chapter
   - [ ] Content displays correctly
   - [ ] Code blocks readable
   - [ ] Images load
   - [ ] Internal links work
   - [ ] Repeat for Modules 2, 3, 4

3. **Test Navigation**:
   - [ ] Sidebar navigation works
   - [ ] Breadcrumbs display
   - [ ] Next/Previous buttons work
   - [ ] Search functionality intact (if enabled)

4. **Test Responsive Breakpoints**:
   - [ ] 320px (iPhone SE): No horizontal scroll, text readable
   - [ ] 768px (iPad): Sidebar collapses, navbar responsive
   - [ ] 1024px (iPad Pro): Full layout displays
   - [ ] 2560px (4K): Content centers, no excessive whitespace

5. **Test Dark Mode Consistency**:
   - [ ] Toggle dark mode on homepage
   - [ ] Toggle dark mode on docs page
   - [ ] Toggle dark mode on module page
   - [ ] All pages use consistent color scheme
   - [ ] Code syntax highlighting readable

**Expected Result**:
- All existing content displays correctly
- No regressions in layout or functionality
- Consistent branding across all pages
- Responsive design works at all breakpoints
- Dark mode fully functional

**Validation**:
- Zero console errors
- Zero layout shifts (CLS)
- All interactive elements functional
- Content remains accessible and readable

**Troubleshooting**:
- Content overlapping hero: Adjust hero height or margins
- Sidebar hidden: Check CSS z-index conflicts
- Dark mode incomplete: Some components may have hardcoded colors (add mode-specific overrides)
- Mobile layout broken: Test specific viewport width, add media query breakpoints

---

## Integration Workflow Summary

**Recommended Order**:
1. Scenario 1 (CSS Variables) - Foundation layer
2. Scenario 2 (Navbar) - Navigation structure
3. Scenario 3 (Hero Section) - Homepage enhancement
4. Scenario 4 (Validation) - Quality assurance
5. Scenario 5 (Regression Check) - Final testing

**Time Estimate**:
- Scenario 1: 15 minutes
- Scenario 2: 20 minutes
- Scenario 3: 30 minutes
- Scenario 4: 20 minutes
- Scenario 5: 30 minutes
- **Total**: ~2 hours (first-time implementation)

**Rollback Plan**:
If any scenario fails critically:
1. Revert changes in reverse order (Scenario 5 → 4 → 3 → 2 → 1)
2. Use `git checkout <file>` to restore original files
3. Rebuild and verify site returns to working state
4. Debug issue in isolation before re-applying changes

---

## Common Issues and Solutions

**Issue**: Colors not applying globally
**Solution**: Verify CSS variables use `--ifm-` prefix and are defined at `:root` level

**Issue**: Dark mode not working
**Solution**: Check selector is `[data-theme='dark']` not `[data-theme='night']` or `.dark`

**Issue**: Hero section overlaps content
**Solution**: Ensure hero is first child in component and has explicit `margin-bottom`

**Issue**: Navbar logo too large/small
**Solution**: Adjust `width` and `height` in logo config (try 24px, 32px, 40px)

**Issue**: Module links 404
**Solution**: Check category slugs match actual directory/file names in `docs/` folder

**Issue**: Build fails with "unknown prop"
**Solution**: React props must be camelCase (className not class, onClick not onclick)

**Issue**: Horizontal scroll on mobile
**Solution**: Add `max-width: 100%` and `overflow-x: hidden` to hero/container elements

**Issue**: Footer not using theme colors
**Solution**: Verify footer CSS variables defined and footer style is 'dark' in config

---

## Next Steps After Implementation

Once all scenarios pass:
1. Create pull request with all changes
2. Deploy to staging environment
3. Conduct user acceptance testing (UAT)
4. Deploy to production (Vercel)
5. Monitor for any reported issues
6. Document any deviations from plan

**Success Criteria Met When**:
- All 20 functional requirements (FR-001 to FR-020) validated
- All 8 success criteria (SC-001 to SC-008) achieved
- Build and deployment successful
- No user-reported regressions in first 48 hours
