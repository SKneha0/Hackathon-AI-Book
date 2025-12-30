import {themes as prismThemes} from 'prism-react-renderer';

const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied Intelligence',
  favicon: 'img/favicon.ico',
  url: 'https://YourGitHubUser.github.io',
  baseUrl: '/',
  organizationName: 'YourGitHubUser',
  projectName: 'Hakathon-ai-book',
  onBrokenLinks: 'warn',

  i18n: { defaultLocale: 'en', locales: ['en'] },

  presets: [
    ['classic', {
      docs: {
        sidebarPath: require.resolve('./sidebars.js'),
        routeBasePath: '/',
      },
     blog: {
  showReadingTime: true,
  blogTitle: 'Latest Posts',
  onInlineAuthors: 'warn',
  blogDescription: 'Read the latest articles from our book community',
  postsPerPage: 5,
  routeBasePath: 'blog',
  path: './blog',
},

      theme: {
        customCss: require.resolve('./src/css/custom.css'),
      },
    }],
  ],

  themeConfig: {
    navbar: {
      logo: { alt: 'Book Logo', src: 'img/nee-logo.png' },
      items: [
           { type: 'docSidebar', sidebarId: 'tutorialSidebar', label: 'Book' },
           { to: '/blog', label: 'Blog', position: 'left' },
           { href: 'https://github.com/SKneha0/Hackathon-AI-Book', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics.`,
    },
    prism: { theme: prismThemes.github, darkTheme: prismThemes.dracula },
  },
};

export default config;
