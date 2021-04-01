module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  collectCoverageFrom: [
    'lib/**/*.{ts,tsx}',
    '!lib/**/*.d.ts',
    '!lib/index.ts',
    '!lib/utils/index.ts',
    '!**/stories/**',
    '!**/tests/**',
  ],
  coveragePathIgnorePatterns: [],
  moduleFileExtensions: ['ts', 'tsx', 'json', 'js', 'jsx', 'node'],
};
