const http = require('http');
const { execSync } = require('child_process');
/**
 * Waits for the authentication server to be ready.
 * @param timeout Max amount of time (in milliseconds) to wait for
 */
async function authReady(timeout = 30000) {
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
      let container = execSync('docker ps -q --filter ancestor=romi-dashboard/auth').toString();
      console.log('This is the container =====>>>> ' + container);
      if (container) {
        console.log('Successuflly created auth container -----------------------');
        process.env.CONTAINER = container;
        execSync('echo $CONTAINER', { stdio: 'inherit' });
        execSync('echo $NETWORK', { stdio: 'inherit' });
        execSync('docker network connect $NETWORK $CONTAINER', { stdio: 'inherit' });
        console.log('=========================== END =============================');
      } else {
        console.log('again ------------------------------------');
        execSync('echo $CONTAINER', { stdio: 'inherit' });
        execSync('echo $NETWORK', { stdio: 'inherit' });
        console.log('=========================== END =============================');
      }
      req = http.request('http://localhost:8080/auth/', () => {
        console.log(
          '-------------------------------- connecting success ------------------------------',
        );
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
