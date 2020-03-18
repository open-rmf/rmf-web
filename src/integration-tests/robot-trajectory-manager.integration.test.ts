import Big from 'big.js';
import * as ChildProcses from 'child_process';
import { RobotTrajectoryManager } from '../robot-trajectory-manager';

// trajectory server is broken atm
describe('robot trajectory manager', () => {
  let rmfDemo: ChildProcses.ChildProcessWithoutNullStreams;
  let scheduleServer: ChildProcses.ChildProcessWithoutNullStreams;

  // FIXME: ros2 launch/run does not clean up child processes
  // beforeAll(() => {
  //   rmfDemo = ChildProcses.spawn('ros2', ['launch', 'demos', 'office.launch.xml']);
  //   scheduleServer = ChildProcses.spawn('ros2', [
  //     'run',
  //     'rmf_schedule_visualizer',
  //     'schedule_visualizer',
  //   ]);
  // });

  // afterAll(async () => {
  //   scheduleServer.kill();
  //   rmfDemo.kill();
  // });

  let trajMan: RobotTrajectoryManager;
  beforeAll(async () => {
    trajMan = await RobotTrajectoryManager.create('ws://localhost:8006');
  });

  it('is able to get trajectory data', async () => {
    expect(await trajMan.latestTrajectory(new Big(6000000000))).toBeTruthy();
  });
});
