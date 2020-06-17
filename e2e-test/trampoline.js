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
}

(async () => {
  // TODO: headless mode for CI
  const officeDemo = child_process.spawn('ros2', ['launch', 'demos', 'office.launch.xml']);
  const soss = child_process.spawn('soss', [`${__dirname}/soss.yaml`]);
  const visualizerServer = child_process.spawn('ros2', ['launch', 'visualizer', 'server.xml']);
  const serve = child_process.spawn('npx', ['serve', 'build']);
  const ros2Echo = child_process.spawn('ros2', [
    'topic',
    'echo',
    'fleet_states',
    'rmf_fleet_msgs/msg/FleetState',
  ]);

  const killProcesses = async () => {
    return Promise.all([
      killProcess(officeDemo, 'SIGINT'), // doesn't clean up properly with SIGTERM
      killProcess(soss),
      killProcess(visualizerServer, 'SIGINT'), // doesn't clean up properly with SIGTERM
      killProcess(serve),
      killProcess(ros2Echo),
    ]);
  };

  process.once('beforeExit', async () => {
    await killProcesses();
  });

  // wait for the rmf to be ready
  await new Promise(res =>
    ros2Echo.stdout.once('data', () => {
      ros2Echo.kill();
      res();
    }),
  );

  const trampProc = child_process.spawnSync(process.argv[2], process.argv.slice(3), {
    stdio: 'inherit',
  });
  process.exitCode = trampProc.status;

  await killProcesses();
})();
