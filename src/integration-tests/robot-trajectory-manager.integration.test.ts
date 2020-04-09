import { DefaultTrajectoryManager } from '../robot-trajectory-manager';

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
        trim: true,
      },
    });
    expect(traj.values.length).toBeTruthy();
    const segments = traj.values[0].segments;
    expect(segments.length).toBeTruthy();
    const segment = segments[0];
    expect(typeof segment.t).toBe('number');
  });

  // regression
  it('is able to get long duration trajectory', async () => {
    const traj = await trajMan.latestTrajectory({
      request: 'trajectory',
      param: {
        map_name: 'L1',
        duration: (2 ** 31) - 1,
        trim: true,
      },
    });
    expect(traj.values.length).toBeTruthy();
    const segments = traj.values[0].segments;
    expect(segments.length).toBeTruthy();
    const segment = segments[0];
    expect(typeof segment.t).toBe('number');
  });

  it('can send simultanenous requests', async () => {
    const results = await Promise.all([
      trajMan.serverTime({
        request: 'time',
        param: {},
      }),
      trajMan.latestTrajectory({
        request: 'trajectory',
        param: {
          map_name: 'L1',
          duration: 6000,
          trim: true,
        },
      }),
    ]);
    expect(results.length).toBe(2);
    expect(typeof results[0].values[0]).toBe('number');
    expect(typeof results[1].values[0]).toBe('object');
  });
});
