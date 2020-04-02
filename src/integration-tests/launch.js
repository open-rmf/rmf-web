#!/usr/bin/env node

const child_process = require('child_process');
const assert = require('assert');

(async () => {
  const officeDemo = child_process.spawn('ros2', ['launch', 'demos', 'office.launch.xml'], {
    stdio: 'inherit',
  });
  const visualizerServer = child_process.spawn('ros2', ['launch', 'visualizer', 'server.xml'], {
    stdio: 'inherit',
  });

  const killProcessPromise = (proc, signal) => {
    if (proc.kill(signal)) {
      return new Promise(res => proc.once('exit', res));
    }
  };

  // how to know if its ready to receive request?
  await new Promise(res => setTimeout(res, 10000));
  assert.equal(
    child_process.spawnSync('ros2', ['launch', 'demos', 'office_loop.launch.xml'], {
      stdio: 'inherit',
    }).status,
    0,
  );

  child_process.spawnSync('node', ['node_modules/.bin/react-scripts', ...process.argv.slice(2)], {
    stdio: 'inherit',
  });

  await Promise.all([
    killProcessPromise(officeDemo, 'SIGINT'),
    killProcessPromise(visualizerServer, 'SIGINT'),
  ]);
})();
