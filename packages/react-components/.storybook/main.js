module.exports = {
  // typescript: {
  //   check: true,
  // },
  framework: {
    name: '@storybook/react-webpack5',
    options: { fastRefresh: true },
  },
  features: {
    storyStoreV7: false,
  },
  stories: ['../lib/**/*.stories.@(js|jsx|ts|tsx)'],
  // stories: ['../lib/**/*.stories.mdx', '../lib/**/*.stories.@(js|jsx|ts|tsx)'],
  // addons: ['@storybook/addon-links', '@storybook/addon-essentials'],
  // webpackFinal: async (config) => {
  //   return {
  //     ...config,
  //     resolve: {
  //       ...config.resolve,
  //       extensions: ['.ts', '.tsx', '.mjs', '.js', '.jsx', '.cjs'],
  //       alias: {
  //         ...config.resolve.alias,
  //         // FIXME - need to let storybook use the latest version of emotion so that
  //         // the theme gets picked up correctly
  //         '@emotion/core': require.resolve('@emotion/react'),
  //         'emotion-theming': require.resolve('@emotion/react'),
  //       },
  //     },
  //   };
  // },
  // babel: async (options) => {
  //   options.plugins.push('@babel/plugin-syntax-flow');
  //   options.presets.push('@babel/preset-typescript');
  //   return options;
  // }
};
