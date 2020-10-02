const { execSync } = require('child_process');

const installCmd = process.env.npm_config_refer === 'ci' ? 'npm ci' : 'npm install';
// avoid name clash
const installed = process.env.__aspidfr__installed
  ? new Set(process.env.__aspidfr__installed.split(';'))
  : new Set();
if (!installed.has('root')) {
  execSync(installCmd, { cwd: `${__dirname}/..`, stdio: 'inherit' });
  installed.add('root');
}
process.argv.slice(2).forEach((package) => {
  if (!installed.has(package)) {
    execSync(installCmd, {
      cwd: `${__dirname}/../packages/${package}`,
      stdio: 'inherit',
      env: {
        ...process.env,
        __aspidfr__installed: Array.from(installed.values())
          .reduce((s, pkg) => (s += `;${pkg}`), '')
          .slice(1),
      },
    });
    installed.add(package);
  }
});
