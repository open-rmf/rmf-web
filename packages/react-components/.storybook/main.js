module.exports = {
  framework: {
    name: '@storybook/react-webpack5',
    options: { fastRefresh: true },
  },
  features: {
    storyStoreV7: false,
  },
  stories: ['../lib/**/*.stories.@(js|jsx|ts|tsx)'],
  core: {
    builder: {
      name: '@storybook/builder-webpack5',
      options: {
        fsCache: true,
        lazyCompilation: true,
      },
    },
  },
};
