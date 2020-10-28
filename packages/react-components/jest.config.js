module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  collectCoverageFrom: ['lib/**/*.{js,jsx,ts,tsx}'],
  coveragePathIgnorePatterns: [
    'lib/color-manager.ts', // uses many apis not available it jsdom
  ],
};
