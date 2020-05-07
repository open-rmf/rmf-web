import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import FakeTrajectoryManager from '../../mock/fake-traj-manager';
import RobotTrajectory from './robot-trajectory';

const mount = createMount();

it('renders without crashing', async () => {
  const trajMgr = new FakeTrajectoryManager();
  const trajectoryData = await trajMgr.latestTrajectory({
    request: 'trajectory',
    param: {
      map_name: 'L1',
      duration: 1000,
      trim: true,
    },
  });
  const trajectoryValue = trajectoryData.values[0];
  const trajectoryConflict = trajectoryData.conflicts;

  const root = mount(
    <svg>
      <RobotTrajectory
        trajectory={trajectoryValue}
        conflicts={trajectoryConflict}
        footprint={0.5}
        color="black"
      />
    </svg>,
  );

  root.unmount();
});
