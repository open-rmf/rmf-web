const { execSync } = require('child_process');

const packages = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');
execSync(`npm install --no-package-lock --package-lock-only ${packages}`, {
  stdio: 'inherit',
});
execSync(`npm install --prefix ${__dirname}/..`, { stdio: 'inherit' });
