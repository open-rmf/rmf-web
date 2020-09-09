import * as http from 'http';

/**
 * Waits for the authentication server to be ready.
 * @param timeout Max amount of time (in milliseconds) to wait for
 */
async function authReady(timeout: number = 30000): Promise<boolean> {
  return new Promise(res => {
    let req: http.ClientRequest | undefined;
    const timer = setTimeout(() => {
      req?.abort();
      clearTimeout(timer);
      clearTimeout(retryTimer);
      res(false);
    }, timeout);
    let retryTimer: NodeJS.Timeout;
    const waitAuthReady = () => {
      req = http.request('http://localhost:8080/auth/', () => {
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
