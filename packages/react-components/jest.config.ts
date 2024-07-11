import { Config } from 'jest';

export default {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  rootDir: 'lib',
  moduleNameMapper: {
    '\\.(css|less)$': 'identity-obj-proxy',
  },
  testPathIgnorePatterns: ['<rootDir>/.*/test-utils.spec.tsx?', '<rootDir>/.*/test-data.spec.tsx?'],
  collectCoverageFrom: ['lib/**/*.{ts,tsx}', '!**/*.stories.*', '!**/*.test.*'],
} as Config;
