module.exports = {
  preset: 'ts-jest',
  rootDir: 'lib',
  testEnvironment: 'jsdom',
  collectCoverageFrom: ['lib/**/*.{ts,tsx}', '!**/*.stories.*', '!**/*.test.*'],
  coveragePathIgnorePatterns: [],
  moduleFileExtensions: ['ts', 'tsx', 'json', 'js', 'jsx', 'node'],
};
