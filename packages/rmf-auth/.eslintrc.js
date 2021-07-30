module.exports = {
  extends: ['../../.eslintrc.js'],
  overrides: [
    {
      files: ['lib/**', 'stories/**', 'tests/**'],
      env: {
        node: false,
      },
    },
  ],
};
