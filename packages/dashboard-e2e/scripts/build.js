const { execSync } = require('child_process');
const path = require('path');

process.env.RMF_DASHBOARD_RESOURCES_FILE = path.resolve(`${__dirname}/../dashboard-resources.json`);
execSync('cd ../dashboard && npm run setup', { stdio: 'inherit' });
execSync('pnpm run --filter {.}^... build', { stdio: 'inherit' });
