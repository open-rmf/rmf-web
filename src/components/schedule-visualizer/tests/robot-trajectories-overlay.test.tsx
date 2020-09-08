import { createShallow } from '@material-ui/core/test-utils';
import React from 'react';
import RobotTrajectoriesOverlay, {
  RobotTrajectoriesOverlayProps,
} from '../robot-trajectories-overlay';
import { Conflict, Trajectory } from '../../../robot-trajectory-manager';
import ColorManager from '../colors';
import FakeTrajectoryManager from '../../../mock/fake-traj-manager';
import { mapBound } from '../../../stories/baseComponents/utils';

const mount = createShallow();

const createWrapper = (
  Component: React.MemoExoticComponent<(props: RobotTrajectoriesOverlayProps) => JSX.Element>,
  bounds: L.LatLngBoundsExpression,
  conflicts: Conflict[],
  colorManager: ColorManager,
  trajs: Trajectory[],
  conflictRobotNames: string[][],
) => {
  return mount(
    <Component
      bounds={bounds}
      conflicts={conflicts}
      colorManager={colorManager}
      trajs={trajs}
      conflictRobotNames={conflictRobotNames}
    />,
  );
};

describe('RobotTrajectoriesOverlay', () => {
  let trajMgr;
  let trajectoryData;
  let trajectoryValue: Trajectory;
  let trajectoryConflict: Conflict[];
  let colorManager: ColorManager;

  beforeEach(async () => {
    colorManager = new ColorManager();
    trajMgr = new FakeTrajectoryManager();
    trajectoryData = await trajMgr.latestTrajectory({
      request: 'trajectory',
      param: {
        map_name: 'L1',
        duration: 1000,
        trim: true,
      },
    });

    trajectoryValue = trajectoryData.values[0];
    trajectoryConflict = trajectoryData.conflicts;
  });

  it('test', () => {
    const wrapper = createWrapper(
      RobotTrajectoriesOverlay,
      mapBound,
      trajectoryConflict,
      colorManager,
      [trajectoryValue],
      [[]],
    );
    wrapper.unmount();
  });
});
