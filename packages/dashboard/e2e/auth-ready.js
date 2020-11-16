const http = require('http');
const { execSync, exec } = require('child_process');
/**
 * Waits for the authentication server to be ready.
 * @param timeout Max amount of time (in milliseconds) to wait for
 */
async function authReady(timeout = 80000) {
  return new Promise((res) => {
    let req;
    const timer = setTimeout(() => {
      req && req.abort();
      clearTimeout(timer);
      clearTimeout(retryTimer);
      res(false);
    }, timeout);
    let retryTimer;
    const waitAuthReady = () => {
      // initialize network and container related variables.
      let authIpAddress;
      const commonNetwork = 'auth_dashboard_network';
      const commonNetworkSubnet = '172.16.0.0/16';
      const authContainerIp = '172.16.0.2';

      // check if we are in github CI environment
      if (process.env.CI) {
        // check if auth container has already been initialized
        let authContainer = execSync(
          'docker ps -q --filter ancestor=romi-dashboard/auth',
        ).toString();
        if (authContainer) {
          process.env.AUTH_CONTAINER = authContainer;

          // check if auth container is already connected to custom network to prevent running the same operation if it has
          let isConnected = execSync(
            `docker ps -q --filter network=${commonNetwork} --filter ancestor=romi-dashboard/auth`,
          ).toString();

          if (!isConnected) {
            // find and set default auth network id
            let defaultAuthNetwork = execSync(
              "docker inspect $AUTH_CONTAINER -f '{{range.NetworkSettings.Networks }}{{.NetworkID}}{{end}}'",
            ).toString();
            process.env.DEFAULT_AUTH_NETWORK = defaultAuthNetwork;

            // create common network with 172.16.xx.x subnet and connect the auth and dashboard container
            // We need to specify a subnet to assign an ip address later
            // use a 172.16 range ip as it is private, like localhost but is not blocked by github's environment
            execSync(`docker network create --subnet=${commonNetworkSubnet} ${commonNetwork}`, {
              stdio: 'inherit',
            });
            execSync(
              `docker network connect --ip=${authContainerIp} ${commonNetwork} $AUTH_CONTAINER`,
              {
                stdio: 'inherit',
              },
            );
            execSync(`docker network connect ${commonNetwork} $ROMIDASHBOARD_CONTAINER`, {
              stdio: 'inherit',
            });

            // disconnect from old network to prevent concatenation of 2 different network ip later on
            execSync(`docker network disconnect $DEFAULT_AUTH_NETWORK $AUTH_CONTAINER`, {
              stdio: 'inherit',
            });
            execSync('docker network disconnect $GITHUB_NETWORK $ROMIDASHBOARD_CONTAINER', {
              stdio: 'inherit',
            });

            // assign of auth container ip
            authIpAddress = authContainerIp;
          }
        }
      }

      req = http.request(`http://${authIpAddress ? authIpAddress : 'localhost'}:8080/auth/`, () => {
        clearTimeout(timer);
        clearTimeout(retryTimer);
        res(true);
      });
      req.once('error', () => {
        retryTimer = setTimeout(waitAuthReady, 20000);
      });
      req.end();
    };
    waitAuthReady();
  });
}

authReady();
