const concurrently = require('concurrently');
const { execSync } = require('child_process');

const keycloakAddress = process.env.E2E_KEYCLOAK_ADDRESS || 'localhost:8088';

process.env.REACT_APP_TRAJECTORY_SERVER = 'ws://localhost:8006';
process.env.REACT_APP_ROS2_BRIDGE_SERVER = 'ws://localhost:50002';
process.env.ROMI_DASHBOARD_PORT = '5000';

execSync('WORLD_NAME=office node scripts/get-resources-location.js', { stdio: 'inherit' });
execSync('cd .. && node ./scripts/setup/get-icons.js', { stdio: 'inherit' });
execSync('npm --prefix .. run build', {
  stdio: 'inherit',
  env: {
    ...process.env,
    REACT_APP_AUTH_PROVIDER: 'keycloak',
    REACT_APP_KEYCLOAK_CONFIG: JSON.stringify({
      realm: 'master',
      clientId: 'romi-dashboard',
      url: `http://${keycloakAddress}/auth`,
    }),
  },
});

// wrap in double quotes to support args with spaces
const wdioArgs = process.argv
  .slice(2)
  .map((arg) => `"${arg}"`)
  .join(' ');

concurrently(
  [
    'npm run start:ros2-bridge',
    'npm run start:keycloak',
    'serve -c ../e2e/serve.json ../build',
    `node scripts/auth-ready.js && wdio ${wdioArgs}`,
  ],
  {
    killOthers: ['success', 'failure'],
    successCondition: 'first',
  },
).catch((e) => {
  console.error(e);
  process.exitCode = -1;
});
