const io = require('socket.io-client');

const uri = process.argv[2];
const path = process.argv[3];

(async () => {
  const client = io(uri);
  const timeout = setTimeout(() => {
    console.log('timed out');
    client.disconnect();
    process.exitCode = 2;
  }, 1000);
  client.on('subscribe', (resp) => {
    console.log(resp);
    client.disconnect();
    clearTimeout(timeout);
    process.exitCode = 1;
  });
  client.on(path, (resp) => {
    console.log(resp);
    client.disconnect();
    clearTimeout(timeout);
  });
  client.emit('subscribe', { path: path });
})();
