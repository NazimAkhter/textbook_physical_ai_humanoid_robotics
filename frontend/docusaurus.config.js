// @ts-check
// JSDoc annotations allow editor autocompletion and type checking
// (when paired with ).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Book',
  tagline: 'Master humanoid robotics with ROS 2, digital twins, and Vision-Language-Action systems',
  favicon: 'https://cdn3d.iconscout.com/3d/premium/thumb/robot-3d-icon-png-download-9911391.png',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'physical-ai-book',
  projectName: 'physical-ai-book',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Book',
          src: 'https://cdn3d.iconscout.com/3d/premium/thumb/robot-3d-icon-png-download-9911391.png',
          href: '/',
          target: '_self',
          width: 32,
          height: 32,
          className: 'navbar__logo',
        },
        items: [
           {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Reading Book',
        },
          {
            href: 'https://github.com/physical-ai-book/physical-ai-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'ROS 2 Nervous System',
                to: '/docs/category/module-1-ros-2-nervous-system',
              },
              {
                label: 'Digital Twins',
                to: '/docs/category/module-2-digital-twins',
              },
              {
                label: 'AI-Robot Brain',
                to: '/docs/category/module-3-ai-robot-brain',
              },
              {
                label: 'VLA Systems',
                to: '/docs/category/module-4-vla-systems',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/physical-ai-book/physical-ai-book',
              },
            ],
          },
        ],
        copyright: 'Copyright © 2025 Physical AI & Humanoid Robotics Book. Built with Docusaurus.',
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
