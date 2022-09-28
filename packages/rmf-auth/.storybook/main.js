module.exports = {
  typescript: {
    check: true,
  },
  stories: ['../lib/**/*.stories.mdx', '../lib/**/*.stories.@(js|jsx|ts|tsx)'],
  addons: ['@storybook/addon-links', '@storybook/addon-essentials'],
  webpackFinal: async (config) => {
    return {
      ...config,
      resolve: {
        ...config.resolve,
        extensions: ['.ts', '.tsx', '.mjs', '.js', '.jsx', '.cjs'],
        alias: {
          ...config.resolve.alias,
        },
      },
    };
  },
};
