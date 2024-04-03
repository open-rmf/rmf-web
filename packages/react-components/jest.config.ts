import { Config } from 'jest';

export default {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  rootDir: 'lib',
  moduleNameMapper: {
    '\\.(css|less)$': 'identity-obj-proxy',
  },
  testPathIgnorePatterns: ['<rootDir>/.*/test-utils.spec.tsx?', '<rootDir>/.*/test-data.spec.tsx?'],
} as Config;
