import { mount } from 'enzyme';
import React from 'react';
import { makeRobot } from '../../../mock/utils';
import ColorManager from '../colors';
import RobotImageIcon from '../robot-image-icon';

describe('RobotImageIcon', () => {
  const colorManager = new ColorManager();
  // TextEncoder is not available in node
  colorManager.robotColor = jest.fn(async () => 'black');
  colorManager.robotColorFromCache = jest.fn(() => 'black');

  const iconPath = 'testIcon';

  test('renders correctly', async () => {
    const root = mount(
      <svg>
        <RobotImageIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          dispatchIconError={jest.fn()}
          iconPath={iconPath}
        />
      </svg>,
    );
    expect(root).toMatchSnapshot();
  });

  test('red shadow appears for conflicting robots', async () => {
    const root = mount(
      <svg>
        <RobotImageIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          dispatchIconError={jest.fn()}
          iconPath={iconPath}
          inConflict={true}
        />
      </svg>,
    );
    expect(root.find('circle#shadow[fill*="conflict"]')).toHaveLength(1);
  });
});
