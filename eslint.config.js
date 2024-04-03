const eslint = require('@eslint/js');
const tseslint = require('typescript-eslint');
const react = require('eslint-plugin-react');
const reactHooks = require('eslint-plugin-react-hooks');

module.exports = tseslint.config(eslint.configs.recommended, ...tseslint.configs.recommended, {
  files: ['**/*.{js,jsx,mjs,cjs,ts,tsx}'],
  plugins: { react, reactHooks },
  settings: {
    react: {
      version: 'detect',
    },
  },
  rules: {
    '@typescript-eslint/no-explicit-any': 'off',
    '@typescript-eslint/ban-types': ['error', { types: { '{}': false }, extendDefaults: true }],
    '@typescript-eslint/no-loss-of-precision': 'off',
  },
});
