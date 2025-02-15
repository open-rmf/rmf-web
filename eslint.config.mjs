// // import { FlatCompat } from '@eslint/eslintrc';
// // import eslint from '@eslint/js';
// import react from 'eslint-plugin-react';
// import reactHooks from 'eslint-plugin-react-hooks';
// import simpleImportSort from 'eslint-plugin-simple-import-sort';
// import storybook from 'eslint-plugin-storybook';
// import tseslint from 'typescript-eslint';

// // const compat = new FlatCompat();

// // export default tseslint.config(
// export default [
//   // eslint.configs.recommended,
//   ...tseslint.configs.recommended,
//   storybook.configs.recommended,
//   {
//     files: ['**/*.{js,jsx,mjs,cjs,ts,tsx}'],
//     // plugins: { react, 'react-hooks': reactHooks },
//     plugins: {
//       'react': react,
//       'react-hooks': reactHooks,
//       'simple-import-sort': simpleImportSort,
//     },
//     settings: {
//       react: {
//         version: 'detect',
//       },
//     },
//     rules: {
//       '@typescript-eslint/no-explicit-any': 'off',
//       '@typescript-eslint/ban-types': ['error', { types: { '{}': false }, extendDefaults: true }],
//       '@typescript-eslint/no-loss-of-precision': 'off',
//       '@typescript-eslint/no-unused-vars': ['warn', { argsIgnorePattern: '^_' }],
//       ...reactHooks.configs.recommended.rules,
//       'simple-import-sort/imports': 'error',
//       'simple-import-sort/exports': 'error',
//     },
//   },
//   // ...compat.config(storybook.configs.recommended),
//   // {
//   //   plugins: { 'simple-import-sort': simpleImportSort },
//   // },
//   // rules: {
//   // },
// ];

import tseslint from 'typescript-eslint';
import react from 'eslint-plugin-react';
import reactHooks from 'eslint-plugin-react-hooks';
import simpleImportSort from 'eslint-plugin-simple-import-sort';
import storybook from 'eslint-plugin-storybook';

export default [
  ...tseslint.configs.recommended,
  // storybook.configs.recommended,
  {
    files: ['**/*.{js,jsx,ts,tsx,mjs,cjs}'],
    ignores: ['node_modules/', 'dist/'],

    languageOptions: {
      ecmaVersion: 'latest',
      sourceType: 'module',
    },

    plugins: {
      react,
      'react-hooks': reactHooks,
      'simple-import-sort': simpleImportSort,
    },

    settings: {
      react: {
        version: 'detect',
      },
    },

    rules: {
      '@typescript-eslint/no-explicit-any': 'off',
      '@typescript-eslint/ban-types': ['error', { types: { '{}': false }, extendDefaults: true }],
      '@typescript-eslint/no-loss-of-precision': 'off',
      '@typescript-eslint/no-unused-vars': ['warn', { argsIgnorePattern: '^_' }],
      // ...reactHooks.configs.recommended.rules,
      'simple-import-sort/imports': 'error',
      'simple-import-sort/exports': 'error',
    },
  },
];
