import * as ChildProcess from 'child_process';

(async () => {
  const rmfDemo = ChildProcess.spawn('ros2', ['launch', 'demos', 'office.launch.xml']);
  const visServer = ChildProcess.spawn('ros2', ['launch', 'visualizer', 'server.xml']);

  // FIXME: anyway to know when it is ready?
  // wait for the services to be ready to receive request
  await new Promise(res => setTimeout(res, 5000));

  ChildProcess.spawnSync('ros2', ['launch', 'demos', 'office_loop.launch.xml']);


  require(`${__dirname}/../../node_modules/.bin/react-scripts`)
})();
