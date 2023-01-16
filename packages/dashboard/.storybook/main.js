module.exports = {
  stories: ['../src/**/*.stories.tsx'],
  addons: [
    {
      name: '@storybook/preset-create-react-app',
      options: {
        scriptsPackageName: 'react-scripts',
      },
    },
    '@storybook/addon-essentials',
    '@storybook/addon-links',
  ],
};
