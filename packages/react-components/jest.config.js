module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  collectCoverageFrom: ['lib/**/*.{ts,tsx}', '!lib/**/*.d.ts'],
  coveragePathIgnorePatterns: [
    'lib/color-manager.ts', // uses many apis not available it jsdom
    'lib/robots/trajectory*', // uses svg methods not supported by jsdom
    'lib/svg-text.ts', // uses computed text length which is not supported by jsdom
  ],
  moduleFileExtensions: ['ts', 'tsx', 'json', 'js', 'jsx', 'node'],
};
