const concurrently = require('concurrently');
const { execSync } = require('child_process');

execSync('node ./scripts/setup/get-icons.js');

execSync('npm run build:e2e', { stdio: 'inherit' });

// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

const targets = ['npm:start:api', 'npm:start:react:e2e', 'npm:start:auth'];

// auth service will be started as a service container in github actions.
//
// When running gh workflow inside a container, gh automatically assigns it to a docker network,
// this causes the auth container spawned in the docker-compose network to not be able to connect
// to the e2e tests.
//
// An alternative is to launch the e2e container manually through docker-compose, but the downside
// is that we can't use gh actions from within the container. This also creates many permission
// problems as the container is running as root, and the files it creates are not accessible
// by the host and some gh actions fails as a result of that. There is also a bug where
// npm is not running as root EUID even though we are running npm as root, which causes some
// commands like git ls-remote to fail, see https://npm.community/t/npm-ignores-unsafe-perm-and-user-params-when-running-git-as-part-of-install/10437
// if (!process.env.CI) {
//   targets.push('npm:start:auth');
// }

concurrently([...targets, `npm:test:e2e:wdio -- ${wdioArgs}`], {
  killOthers: ['success', 'failure'],
  successCondition: 'first',
}).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
