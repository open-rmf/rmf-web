module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  rootDir: 'lib',
  collectCoverageFrom: ['lib/**/*.{ts,tsx}', '!**/*.test.*'],
  coveragePathIgnorePatterns: [],
  moduleFileExtensions: ['ts', 'tsx', 'json', 'js', 'jsx', 'node'],
};
