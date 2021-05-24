const concurrently = require('concurrently');
const { execSync } = require('child_process');

process.env.REACT_APP_AUTH_PROVIDER = process.env.REACT_APP_AUTH_PROVIDER || 'keycloak';
if (process.env.REACT_APP_AUTH_PROVIDER === 'keycloak') {
  process.env.REACT_APP_KEYCLOAK_CONFIG =
    process.env.REACT_APP_KEYCLOAK_CONFIG ||
    JSON.stringify({
      realm: 'master',
      clientId: 'rmf-dashboard',
      url: 'http://localhost:8088/auth',
    });
}
process.env.REACT_APP_TRAJECTORY_SERVER =
  process.env.REACT_APP_TRAJECTORY_SERVER || 'ws://localhost:8006';
process.env.REACT_APP_RMF_SERVER = process.env.REACT_APP_RMF_SERVER || 'http://localhost:8000';
process.env.E2E_USER = process.env.E2E_USER || 'admin';
process.env.E2E_PASSWORD = process.env.E2E_PASSWORD || 'admin';
process.env.E2E_DASHBOARD_URL = process.env.E2E_DASHBOARD_URL || 'http://localhost:5000';

execSync('WORLD_NAME=office node scripts/get-resources-location.js', { stdio: 'inherit' });
execSync('cd ../ && node ./dashboard/scripts/setup/get-icons.js', { stdio: 'inherit' });
execSync('npm --prefix ../dashboard run build', { stdio: 'inherit' });

// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

const services = [];
// eslint-disable-next-line no-eval
if (!eval(process.env.E2E_NO_AUTH)) {
  switch (process.env.REACT_APP_AUTH_PROVIDER) {
    case 'keycloak':
      services.push('npm run start:keycloak');
      break;
    default:
      throw new Error('cannot start auth provider');
  }
}
// eslint-disable-next-line no-eval
if (!eval(process.env.E2E_NO_DASHBOARD)) {
  services.push('serve -c ../serve.json build');
}
// eslint-disable-next-line no-eval
if (!eval(process.env.E2E_NO_RMF_SERVER)) {
  services.push('npm run start:rmf-server');
}

concurrently([...services, `node scripts/auth-ready.js && wdio ${wdioArgs}`], {
  killOthers: ['success', 'failure'],
  successCondition: 'first',
}).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
