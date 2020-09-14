import { mount } from 'enzyme';
import React from 'react';
import { makeRobot } from '../../../mock/utils';
import ColorManager from '../colors';
import RobotDefaultIcon from '../robot-default-icon';

describe('RobotDefaultIcon', () => {
  const colorManager = new ColorManager();
  // TextEncoder is not available in node
  colorManager.robotColor = jest.fn(async () => 'black');
  colorManager.robotColorFromCache = jest.fn(() => 'black');

  test('renders correctly', async () => {
    const root = mount(
      <svg>
        <RobotDefaultIcon robot={makeRobot()} footprint={1} colorManager={colorManager} />
      </svg>,
    );
    expect(root).toMatchSnapshot();
  });

  test('red shadow appears for conflicting robots', async () => {
    const root = mount(
      <svg>
        <RobotDefaultIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          inConflict={true}
        />
      </svg>,
    );
    expect(root.find('circle#shadow[fill*="conflict"]')).toHaveLength(1);
  });
});
