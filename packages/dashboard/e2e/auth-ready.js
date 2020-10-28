const http = require('http');
const { execSync } = require('child_process');
const { executionAsyncId } = require('async_hooks');
/**
 * Waits for the authentication server to be ready.
 * @param timeout Max amount of time (in milliseconds) to wait for
 */
async function authReady(timeout = 30000) {
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
      req = http.request('http://localhost:8080/auth/', () => {
        clearTimeout(timer);
        clearTimeout(retryTimer);
        execSync('echo $IMAGE', { stdio: 'inherit' });
        execSync(
          "export NETWORK=docker inspect $IMAGE --format='{{range $k,$v := .NetworkSettings.Networks }} {{$k}} {{end}}'",
          { stdio: 'inherit' },
        );
        execSync('echo $NETWORK', { stdio: 'inherit' });
        res(true);
      });
      req.once('error', () => {
        retryTimer = setTimeout(waitAuthReady, 1000);
      });
      req.end();
    };
    waitAuthReady();
  });
}

authReady();
