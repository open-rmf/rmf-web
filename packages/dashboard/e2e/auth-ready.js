const http = require('http');
const { execSync } = require('child_process');
/**
 * Waits for the authentication server to be ready.
 * @param timeout Max amount of time (in milliseconds) to wait for
 */
async function authReady(timeout = 60000) {
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
      let authIpAddress;
      console.log('========================== waiting waiting waiting ==========================');

      if (container) {
        console.log('Successuflly created auth container ----------------------- ' + container);

        process.env.CONTAINER = container;

        let isConnected = execSync(
          'docker ps -q --filter network=test-net --filter ancestor=romi-dashboard/auth',
        ).toString();

        authIpAddress = execSync(
          "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $CONTAINER",
        ).toString();
        process.env.AUTH_IP = authIpAddress;

        console.log('auth ip address >>>>>>> ' + authIpAddress + ' ' + typeof process.env.AUTH_IP);

        if (!isConnected) {
          console.log('I am inside isConnected!!! >>>>> ' + isConnected);
          execSync('docker network create test-net', { stdio: 'inherit' });
          execSync('docker network connect test-net $CONTAINER', {
            stdio: 'inherit',
          });
          // execSync('docker network connect test-net $OTHERCONTAINER', {
          //   stdio: 'inherit',
          // });
          execSync('docker network disconnect romidashboarde2e_default $CONTAINER', {
            stdio: 'inherit',
          });
        }

        console.log('=========================== END =============================');
      } else {
        console.log('again ------------------------------------');
        console.log('=========================== END =============================');
      }

      req = http.request(
        `http://${process.env.AUTH_IP ? process.env.AUTH_IP : 'localhost'}:8080/auth/`,
        () => {
          console.log(
            '-------------------------------- connecting success ------------------------------' +
              process.env.AUTH_IP,
          );
          clearTimeout(timer);
          clearTimeout(retryTimer);
          res(true);
        },
      );
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
