const concurrently = require('concurrently');
const { execSync } = require('child_process');

execSync('cd .. && node ./scripts/setup/get-icons.js', { stdio: 'inherit' });

execSync('npm run build', { stdio: 'inherit' });
console.log(
  'starting concurrently ....................................................................',
);
// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

concurrently(
  [
    'cd .. && npm run start:api',
    'cd .. && npm run start:auth',
    'npm:start:react',
    `node auth-ready.js && wdio ${wdioArgs}`,
  ],
  {
    killOthers: ['success', 'failure'],
    successCondition: 'first',
  },
).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
