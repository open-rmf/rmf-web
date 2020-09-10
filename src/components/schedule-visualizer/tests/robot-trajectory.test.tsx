import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import { createMuiTheme } from '@material-ui/core';

import FakeTrajectoryManager from '../../../mock/fake-traj-manager';
import RobotTrajectory, { RobotTrajectoryProps, fixTrajectoryDiameter } from '../robot-trajectory';
import { Trajectory, Conflict } from '../../../robot-trajectory-manager';
import { TrajectoryColor, TrajectoryDiameter, defaultSettings, Settings } from '../../../settings';
import { SettingsContext } from '../../app-contexts';
import toJson from 'enzyme-to-json';

const mount = createMount();

const createWrapper = (
  Component: React.ForwardRefExoticComponent<RobotTrajectoryProps>,
  trajectory: Trajectory,
  trajectoryConflict: Conflict[],
  footprint: number,
  color: string,
  settings?: Settings,
) => {
  return mount(
    <svg>
      {settings ? (
        <SettingsContext.Provider value={settings}>
          <Component
            trajectory={trajectory}
            conflicts={trajectoryConflict}
            footprint={footprint}
            color={color}
          />
        </SettingsContext.Provider>
      ) : (
        <Component
          trajectory={trajectory}
          conflicts={trajectoryConflict}
          footprint={footprint}
          color={color}
        />
      )}
    </svg>,
  );
};

describe('Robot Trajectory', () => {
  let trajMgr;
  let trajectoryData;
  let trajectoryValue: Trajectory;
  let trajectoryConflict: Conflict[];
  const theme = createMuiTheme();
  const themeColor = theme.palette.success.main;

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

  it('renders without crashing', () => {
    const root = createWrapper(RobotTrajectory, trajectoryValue, trajectoryConflict, 0.5, 'black');
    expect(toJson(root)).toMatchSnapshot();
    root.unmount();
  });

  it('should set width to fix value and color to theme color', () => {
    const mockContext = {
      ...defaultSettings(),
      trajectoryDiameter: TrajectoryDiameter.FixSize,
      trajectoryColor: TrajectoryColor.Theme,
    };
    const root = createWrapper(
      RobotTrajectory,
      trajectoryValue,
      trajectoryConflict,
      0.5,
      'black',
      mockContext,
    );

    const trajWidth = root
      .find('path')
      .find('#robotTrajectoryPath')
      .props().strokeWidth;
    const trajColor = root
      .find('path')
      .find('#robotTrajectoryPath')
      .props().stroke;

    expect(trajWidth).toEqual(fixTrajectoryDiameter);
    expect(trajColor).toEqual(themeColor);

    root.unmount();
  });

  it('should render conflicting path component when trajectories are conflicted', () => {
    const mockConflict = [[trajectoryValue.id]];
    const root = createWrapper(RobotTrajectory, trajectoryValue, mockConflict, 0.5, 'black');
    const errorPath = root.find('path').find('#errorPath');

    expect(errorPath).toHaveLength(1);

    root.unmount();
  });
});
