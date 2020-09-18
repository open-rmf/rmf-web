import React from 'react';
import { shallow } from 'enzyme';

import {
  withFillAnimation,
  withFollowAnimation,
  withOutlineAnimation,
} from '../trajectory-animations';
import RobotTrajectory from '../robot-trajectory';
import ColorManager from '../colors';
import FakeTrajectoryManager from '../../../mock/fake-traj-manager';
import { Trajectory, Conflict } from '../../../robot-trajectory-manager';

const footprint = 0.5;
const colorManager = new ColorManager();

describe('withFillAnimation', () => {
  let trajMgr;
  let trajectoryData;
  let trajectoryValue: Trajectory;
  let trajectoryConflict: Conflict[];

  beforeEach(async () => {
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

  it('renders trajectory with fillAnim animations without crashing', () => {
    const FillAnim = withFillAnimation(RobotTrajectory, 1000);
    const wrapper = shallow(
      <FillAnim
        trajectory={trajectoryValue}
        conflicts={trajectoryConflict}
        footprint={footprint}
        colorManager={colorManager}
      />,
    );
    wrapper.unmount();
  });

  it('renders trajectory with followAnim animations without crashing', () => {
    const FollowAnim = withFollowAnimation(RobotTrajectory, 1000);
    const wrapper = shallow(
      <FollowAnim
        trajectory={trajectoryValue}
        conflicts={trajectoryConflict}
        footprint={footprint}
        colorManager={colorManager}
      />,
    );
    wrapper.unmount();
  });

  it('renders trajectory with outlineAnim animations without crashing', () => {
    const OutlineAnim = withOutlineAnimation(RobotTrajectory, 1000);
    const wrapper = shallow(
      <OutlineAnim
        trajectory={trajectoryValue}
        conflicts={trajectoryConflict}
        footprint={footprint}
        colorManager={colorManager}
      />,
    );
    wrapper.unmount();
  });
});
