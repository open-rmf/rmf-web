const { execSync } = require('child_process');

const installCmd = process.env.npm_config_refer === 'ci' ? 'npm ci' : 'npm install';
if (!process.env.__installScript) {
  execSync(installCmd, { cwd: `${__dirname}/..`, stdio: 'inherit' });
}
process.argv.slice(2).forEach((package) => {
  execSync(installCmd, {
    cwd: `${__dirname}/../packages/${package}`,
    stdio: 'inherit',
    env: { ...process.env, __installScript: true },
  });
});
