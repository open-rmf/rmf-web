import { mount, ReactWrapper } from 'enzyme';
import React from 'react';
import { makeRobot } from '../../../mock/utils';
import ColorManager from '../colors';
import RobotDefaultIcon from '../robot-default-icon';
import { act } from 'react-dom/test-utils';

describe('RobotDefaultIcon', () => {
  const colorManager = new ColorManager();
  const fleetName = 'testFleet';

  test('renders correctly', () => {
    const root = mount(
      <svg>
        <RobotDefaultIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          fleetName={fleetName}
        />
      </svg>,
    );
    expect(root).toMatchSnapshot();
  });

  test('red shadow appears for conflicting robots', () => {
    const root = mount(
      <svg>
        <RobotDefaultIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          inConflict={true}
          fleetName={fleetName}
        />
      </svg>,
    );
    expect(root.find('circle#shadow[fill*="conflict"]')).toHaveLength(1);
  });
});
