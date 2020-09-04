import { createMount } from '@material-ui/core/test-utils';
import React from 'react';

import FakeTrajectoryManager from '../../../mock/fake-traj-manager';
import RobotTrajectory, { RobotTrajectoryProps } from '../robot-trajectory';
import { Trajectory, Conflict } from '../../../robot-trajectory-manager';
import { TrajectoryColor, TrajectoryDiameter, defaultSettings, Settings } from '../../../settings';
import { SettingsContext } from '../../app-contexts';

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
  const themeColor = '#4caf50';
  const fixPathSize = 0.4;

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

    root.unmount();
  });

  it('should set width to fix value and color to theme color', () => {
    const mockContext = {
      ...defaultSettings(),
      trajectoryDiameter: TrajectoryDiameter.Fix_size,
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
      .find('#myPath')
      .props().strokeWidth;
    const trajColor = root
      .find('path')
      .find('#myPath')
      .props().stroke;

    expect(trajWidth).toEqual(fixPathSize);
    expect(trajColor).toEqual(themeColor);

    root.unmount();
  });

  it('should set color to a certain shade of green', () => {
    const mockContext = {
      ...defaultSettings(),
      trajectoryColor: TrajectoryColor.Shades,
    };
    const root = createWrapper(
      RobotTrajectory,
      trajectoryValue,
      trajectoryConflict,
      0.5,
      'black',
      mockContext,
    );

    root.unmount();
  });

  it('should change path color to conflicting color', () => {
    const mockConflict = [[trajectoryValue.id]];
    const root = createWrapper(RobotTrajectory, trajectoryValue, mockConflict, 0.5, 'black');
    const errorPath = root.find('path').find('#errorPath');

    expect(errorPath).toHaveLength(1);

    root.unmount();
  });
});
