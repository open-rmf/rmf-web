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

      if (container) {
        console.log('Successuflly created auth container -----------------------');

        process.env.CONTAINER = container;

        let isConnected = execSync(
          'docker ps -q --filter network=test-net --filter ancestor=romi-dashboard/auth',
        ).toString();
        console.log('i am isConnected >>>>> ' + isConnected);

        console.log('auth container ip');
        execSync(
          "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $CONTAINER",
          { stdio: 'inherit' },
        );
        console.log('Github container ip');
        execSync(
          "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $OTHERCONTAINER",
          { stdio: 'inherit' },
        );

        if (!isConnected) {
          console.log('I am inside isConnected!!! >>>>> ' + isConnected);
          execSync('docker network create --subnet 127.0.0.1/16 --ip-range 127.0.0.1/1 test-net', {
            stdio: 'inherit',
          });

          execSync('docker network connect --ip 127.0.0.1 test-net $CONTAINER', {
            stdio: 'inherit',
          });
          execSync('docker network connect test-net $OTHERCONTAINER', {
            stdio: 'inherit',
          });
        }

        //   console.log('=========================== END =============================');
      } else {
        console.log('again ------------------------------------');
        execSync('echo $CONTAINER', { stdio: 'inherit' });
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
      req.once('error', (err) => {
        console.log(err);
        retryTimer = setTimeout(waitAuthReady, 1000);
      });
      req.end();
    };
    waitAuthReady();
  });
}

authReady();
