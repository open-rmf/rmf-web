import { createMount } from '@material-ui/core/test-utils';
import React from 'react';

import FakeTrajectoryManager from '../../../mock/fake-traj-manager';
import RobotTrajectory from '../robot-trajectory';
import { Trajectory, Conflict } from '../../../robot-trajectory-manager';
import {
  SettingsContext,
  TrajectoryColor,
  TrajectoryDiameter,
  defaultSettings,
} from '../../../settings';
import ColorManager from '../colors';

const mount = createMount();

describe('Robot Trajectory', () => {
  let trajMgr;
  let trajectoryData;
  let trajectoryValue: Trajectory;
  let trajectoryConflict: Conflict[];
  const themeColor = '#4caf50';
  const defaultErrorColor = '#f44336';
  const fixPathSize = 0.4;
  const colorManager = new ColorManager();

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

  it('should set width to fix value and color to theme color', () => {
    const mockContext = {
      ...defaultSettings(),
      trajectoryDiameter: TrajectoryDiameter.Fix_size,
      trajectoryColor: TrajectoryColor.Theme,
    };
    const root = mount(
      <svg>
        <SettingsContext.Provider value={mockContext}>
          <RobotTrajectory
            trajectory={trajectoryValue}
            conflicts={trajectoryConflict}
            footprint={0.5}
            color="black"
          />
        </SettingsContext.Provider>
      </svg>,
    );
    const trajWidth = root.find('path').props().strokeWidth;
    const trajColor = root.find('path').props().stroke;

    expect(trajWidth).toEqual(fixPathSize);
    expect(trajColor).toEqual(themeColor);

    root.unmount();
  });

  it('should set color to a certain shade of green', () => {
    const mockContext = {
      ...defaultSettings(),
      trajectoryColor: TrajectoryColor.Shades,
    };
    const root = mount(
      <svg>
        <SettingsContext.Provider value={mockContext}>
          <RobotTrajectory
            trajectory={trajectoryValue}
            conflicts={trajectoryConflict}
            footprint={0.5}
            color="black"
          />
        </SettingsContext.Provider>
      </svg>,
    );
    console.log(root.debug());
    root.unmount();
  });

  it('should change path color to conflicting color', () => {
    const mockConflict = [[trajectoryValue.id]];
    const root = mount(
      <svg>
        <RobotTrajectory
          trajectory={trajectoryValue}
          conflicts={mockConflict}
          footprint={0.5}
          color="black"
        />
      </svg>,
    );
    const trajColor = root.find('path').props().stroke;
    expect(trajColor).toEqual(defaultErrorColor);
    root.unmount();
  });
});
