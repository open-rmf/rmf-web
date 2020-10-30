module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  collectCoverageFrom: ['lib/**/*.{ts,tsx}', '!lib/**/*.d.ts'],
  coveragePathIgnorePatterns: [
    'lib/color-manager.ts', // uses many apis not available it jsdom
  ],
  moduleFileExtensions: ['ts', 'tsx', 'json', 'js', 'jsx', 'node'],
};
