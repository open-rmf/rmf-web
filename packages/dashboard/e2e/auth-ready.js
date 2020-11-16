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

      // check if auth container has already been initialized
      let authContainer = execSync('docker ps -q --filter ancestor=romi-dashboard/auth').toString();
      if (authContainer) {
        process.env.AUTH_CONTAINER = authContainer;

        // check if auth container is already connected to custom network to prevent running the same operation if it has
        let isConnected = execSync(
          `docker ps -q --filter network=${commonNetwork} --filter ancestor=romi-dashboard/auth`,
        ).toString();

        if (!isConnected) {
          console.log('I am inside isConnected!!! >>>>> ' + isConnected);
          let defaultAuthNetwork = execSync(
            "docker inspect $AUTH_CONTAINER -f '{{range.NetworkSettings.Networks }}{{.NetworkID}}{{end}}'",
          ).toString();
          console.log('defaultAuthNetwork: ----------->: ' + defaultAuthNetwork);
          process.env.DEFAULT_AUTH_NETWORK = defaultAuthNetwork;

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
          execSync(`docker network disconnect $DEFAULT_AUTH_NETWORK $AUTH_CONTAINER`, {
            stdio: 'inherit',
          });
          execSync('docker network disconnect $GITHUB_NETWORK $ROMIDASHBOARD_CONTAINER', {
            stdio: 'inherit',
          });

          authIpAddress = execSync(
            "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $AUTH_CONTAINER",
          ).toString();
          process.env.AUTH_IP = authIpAddress;
          console.log(
            'auth ip address >>>>>>> ' + authIpAddress + ' ' + typeof process.env.AUTH_IP,
          );
        }

        console.log('=========================== END =============================');
      } else {
        console.log('again ------------------------------------');
        console.log('=========================== END =============================');
      }

      req = http.request(`http://${authIpAddress ? authIpAddress : 'localhost'}:8080/auth/`, () => {
        console.log(
          '-------------------------------- connecting success ------------------------------' +
            process.env.AUTH_IP,
        );
        execSync(`docker network inspect ${commonNetwork}`, { stdio: 'inherit' });
        clearTimeout(timer);
        clearTimeout(retryTimer);
        res(true);
      });
      req.once('error', (err) => {
        console.log(err);
        retryTimer = setTimeout(waitAuthReady, 20000);
      });
      req.end();
    };
    waitAuthReady();
  });
}

authReady();
