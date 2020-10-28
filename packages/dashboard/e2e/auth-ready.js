const http = require('http');
const { execSync } = require('child_process');
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
        console.log(
          '-------------------------------- connecting success ------------------------------',
        );
        execSync('echo $IMAGE');
        execSync(
          "export NETWORK=$(docker inspect $IMAGE --format='{{range $k,$v := .NetworkSettings.Networks }} {{$k}} {{end}}')",
        );
        execSync('echo $NETWORk');
        console.log('------------------- end ----------------------------');
        execSync('echo ---------------------------- just testing -----------------------');
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
