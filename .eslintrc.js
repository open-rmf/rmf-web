module.exports = {
  env: {
    node: true,
    es2021: true,
  },
  extends: ['eslint:recommended'],
  ignorePatterns: ['*.d.ts'],
  overrides: [
    {
      files: ['*.ts', '*.tsx'],
      extends: ['plugin:@typescript-eslint/recommended'],
      parser: '@typescript-eslint/parser',
      plugins: ['@typescript-eslint'],
      rules: {
        '@typescript-eslint/ban-types': ['error', { types: { '{}': false }, extendDefaults: true }],
      },
    },
    {
      files: ['*.tsx'],
      env: {
        browser: true,
        es2021: true,
      },
      extends: ['plugin:react/recommended', 'plugin:react-hooks/recommended'],
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
      files: ['*.test.ts', '*.test.tsx', '*.spec.ts', '*.spec.tsx'],
      rules: {
        // null assertions is often useful in tests
        '@typescript-eslint/no-non-null-assertion': 'off',
      },
    },
  ],
};
