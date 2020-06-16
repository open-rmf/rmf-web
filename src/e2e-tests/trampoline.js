#!/usr/bin/env node

/**
 * This script starts the required RMF systems and runs the given command. The RMF systems are
 * shutdown when the program finishes.
 *
 * Usage:
 *   node trampoline.js <command> [args]...
 */
require('./prepare');
const child_process = require('child_process');

async function killProcess(proc, signal) {
  if (proc.killed) {
    return Promise.resolve();
  }
  if (proc.kill(signal)) {
    return new Promise(res => proc.once('exit', res));
  }
  console.warn(`failed to kill process ${proc.pid}`);
  return Promise.resolve();
};

(async () => {
  // TODO: headless mode for CI
  const officeDemo = child_process.spawn('ros2', ['launch', 'demos', 'office.launch.xml'], {
    stdio: 'inherit',
  });
  const soss = child_process.spawn('soss', [`${__dirname}/soss.yaml`], {
    stdio: 'inherit',
  });
  const visualizerServer = child_process.spawn('ros2', ['launch', 'visualizer', 'server.xml'], {
    stdio: 'inherit',
  });

  const killProcesses = async () => {
    return Promise.all([
      killProcess(officeDemo, 'SIGINT'), // doesn't clean up properly with SIGTERM
      killProcess(soss),
      killProcess(visualizerServer, 'SIGINT'), // doesn't clean up properly with SIGTERM
    ]);
  };

  process.once('beforeExit', async () => {
    await killProcesses();
  });

  child_process.spawnSync(process.argv[2], process.argv.slice(3), {
    stdio: 'inherit',
  });

  await killProcesses();
})();