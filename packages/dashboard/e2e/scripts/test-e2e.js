const concurrently = require('concurrently');
const { execSync } = require('child_process');

execSync('docker network create auth_network', { stdio: 'inherit' });
const defaultAuthGatewayIp = execSync(
  "docker network inspect -f '{{range .IPAM.Config}}{{.Gateway}}{{end}}' auth_network",
).toString();
process.env.AUTH_GATEWAY_IP = defaultAuthGatewayIp;
// process.env.REACT_APP_AUTH_CONFIG=`{"realm":"master", "clientId":"romi-dashboard", "url":"http://${defaultAuthGatewayIp}:8080/auth"}`
console.log('gateway ip address: ' + process.env.AUTH_GATEWAY_IP);

execSync('cd .. && node ./scripts/setup/get-icons.js', { stdio: 'inherit' });
console.log(process.env.REACT_APP_AUTH_CONFIG);
execSync('npm run build', { stdio: 'inherit' });
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
