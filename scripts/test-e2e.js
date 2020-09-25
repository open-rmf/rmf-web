const concurrently = require('concurrently');
const { execSync } = require('child_process');

execSync('npm run build:e2e', { stdio: 'inherit' });

// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

concurrently(
  ['npm:start:api', 'npm:start:auth', 'npm:start:react:e2e', `npm:test:e2e:wdio -- ${wdioArgs}`],
  {
    killOthers: ['success', 'failure'],
    successCondition: 'first',
  },
).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
