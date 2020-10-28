const http = require('http');
const { execSync } = require('child_process');
/**
 * Waits for the authentication server to be ready.
 * @param timeout Max amount of time (in milliseconds) to wait for
 */
async function authReady(timeout = 300000) {
  console.log('........ auth Ready ..................');
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
      console.log('............... trying waitAuthReady ...................');
      req = http.request('http://localhost:8080/auth/', () => {
        console.log(
          '-------------------------------- connecting success ------------------------------',
        );
        let network = execSync(
          "docker inspect $IMAGE --format='{{range $k,$v := .NetworkSettings.Networks }} {{$k}} {{end}}'",
          { stdio: 'inherit' },
        );
        process.env.NETWORK = network;
        execSync('echo $NETWORK', { stdio: 'inherit' });
        clearTimeout(timer);
        clearTimeout(retryTimer);
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
