module.exports = {
  extends: ['../../.eslintrc'],
  overrides: [
    {
      files: ['lib/**', 'tests/**', 'stories/**'],
      extends: ['plugin:react/recommended', 'plugin:react-hooks/recommended'],
      env: {
        browser: true,
        es2021: true,
      },
      parserOptions: {
        ecmaFeatures: {
          jsx: true,
        },
        ecmaVersion: 12,
        sourceType: 'module',
      },
      plugins: ['react'],
      settings: {
        react: {
          version: 'detect',
        },
      },
      rules: {
        'react/display-name': 'off',
        'react/prop-types': 'off',
      },
    },
    {
      files: 'tests/**',
      rules: {
        '@typescript-eslint/no-non-null-assertion': 'off',
      },
    },
  ],
};
