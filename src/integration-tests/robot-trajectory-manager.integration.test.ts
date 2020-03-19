import Big from 'big.js';
import * as ChildProcses from 'child_process';
import { DefaultTrajectoryManager } from '../robot-trajectory-manager';

// trajectory server is broken atm
describe('robot trajectory manager', () => {
  let trajMan: DefaultTrajectoryManager;
  let rmfDemo: ChildProcses.ChildProcessWithoutNullStreams;

  beforeAll(async () => {
    rmfDemo = ChildProcses.spawn('ros2', ['launch', 'demos', 'office.launch.xml']);
    await new Promise(res => {
      const interval = setInterval(async () => {
        try {
          trajMan = await DefaultTrajectoryManager.create('ws://localhost:8006');
          clearInterval(interval);
          res();
        } catch (e) {
          // do nothing, try again later
        }
      }, 1000);
    });
  }, 30000);

  afterAll(async () => {
    await new Promise(res => {
      // ros2 launch only cleans up children with SIGINT, SIGTERM doesn't work
      rmfDemo.kill('SIGINT');
      rmfDemo.once('exit', res);
    });
  }, Infinity);

  it('is able to get trajectory data', async () => {
    await trajMan.latestTrajectory(new Big(6000000000));
  });
});
