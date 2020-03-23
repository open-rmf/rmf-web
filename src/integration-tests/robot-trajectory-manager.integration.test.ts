import * as FileSystem from 'fs';
import { DefaultTrajectoryManager } from '../robot-trajectory-manager';

// trajectory server is broken atm
describe('robot trajectory manager', () => {
  let trajMan: DefaultTrajectoryManager;

  beforeAll(async () => {
    trajMan = await DefaultTrajectoryManager.create('ws://localhost:8006');
  });

  it('is able to get trajectory data', async () => {
    const traj = await trajMan.latestTrajectory({
      request: 'trajectory',
      param: {
        map_name: 'L1',
        duration: 6000,
      },
    });
    expect(traj.values.length).toBeTruthy();
    const segments = traj.values[0].segments;
    expect(segments.length).toBeTruthy();
    const segment = segments[0];
    // expect(typeof segment.t).toBe('number');
    segments.forEach(seg => {
      seg.t = ~~(parseInt(seg.t as any) / 1000000);
    });
    FileSystem.writeFileSync(`${__dirname}/../../trajectories.json`, JSON.stringify(traj));
  });
});
