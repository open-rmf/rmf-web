import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import FakeTrajectoryManager from '../../mock/fake-traj-manager';
import RobotTrajectory from './robot-trajectory';

const mount = createMount();

it('renders without crashing', async () => {
  const trajMgr = new FakeTrajectoryManager();
  const trajectory = (
    await trajMgr.latestTrajectory({
      request: 'trajectory',
      param: {
        map_name: 'L1',
        duration: 1000,
        trim: true,
      },
    })
  ).values[0];

  const root = mount(
    <svg>
      <RobotTrajectory trajectory={trajectory} footprint={0.5} color="black" />
    </svg>,
  );

  root.unmount();
});
