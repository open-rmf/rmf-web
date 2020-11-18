const http = require('http');
const { execSync } = require('child_process');
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
      const authIpAddress = process.env.AUTH_GATEWAY_IP;

      // check if we are in github CI environment
      if (process.env.CI) {
        // check if auth container has already been initialized
        const authContainer = execSync(
          'docker ps -q --filter ancestor=romi-dashboard/auth',
        ).toString();
        if (authContainer) {
          process.env.AUTH_CONTAINER = authContainer;
          // check if auth container is already connected to custom network
          // to prevent running the same operations if it has already been connected to custom network
          const isConnected = execSync(
            `docker ps -q --filter network=auth_network --filter ancestor=romi-dashboard/auth`,
          ).toString();

          if (!isConnected) {
            execSync(`docker network connect auth_network $ROMIDASHBOARD_CONTAINER`, {
              stdio: 'inherit',
            });
            execSync('docker network disconnect $GITHUB_NETWORK $ROMIDASHBOARD_CONTAINER', {
              stdio: 'inherit',
            });
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
