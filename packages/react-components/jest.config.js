module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  collectCoverageFrom: ['lib/**/*.{js,jsx,ts,tsx}', '!lib/**/tests/*'],
};
