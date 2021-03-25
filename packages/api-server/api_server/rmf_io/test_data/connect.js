const io = require('socket.io-client');

const uri = process.argv[2];
const token = process.argv[3];

console.log(process.argv);

(async () => {
  const client = io(uri, { auth: { token: token } });
  const success = await new Promise((res) => {
    client.on('connect_error', () => res(false));
    client.on('connect', () => res(true));
  });
  client.disconnect();

  if (!success) {
    console.log('fail');
    process.exitCode = 1;
  } else {
    console.log('success');
  }
})();
