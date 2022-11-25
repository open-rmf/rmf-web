const concurrently = require('concurrently');
const { execSync } = require('child_process');
const { services } = require('./config');
const path = require('path');

process.env.RMF_DASHBOARD_RESOURCES_FILE = path.resolve(`${__dirname}/../dashboard-resources.json`);
execSync('cd ../dashboard && npm run setup', { stdio: 'inherit' });
execSync('pnpm run --filter {.}^... build', { stdio: 'inherit' });

// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

concurrently([...services, `wdio ${wdioArgs}`], {
  killOthers: ['success', 'failure'],
  successCondition: 'first',
  prefix: 'none',
}).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
