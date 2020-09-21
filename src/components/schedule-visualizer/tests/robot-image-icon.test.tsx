import { mount } from 'enzyme';
import React from 'react';
import { makeRobot } from '../../../mock/utils';
import ColorManager from '../colors';
import RobotImageIcon from '../robot-image-icon';
import { act } from 'react-dom/test-utils';

describe('RobotImageIcon', () => {
  const colorManager = new ColorManager();

  const iconPath = 'testIcon';
  const fleetName = 'testFleet';

  test('renders correctly', async () => {
    const root = mount(
      <svg>
        <RobotImageIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          dispatchIconError={jest.fn()}
          iconPath={iconPath}
          fleetName={fleetName}
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
          fleetName={fleetName}
        />
      </svg>,
    );
    expect(root.find('circle#shadow[fill*="conflict"]')).toHaveLength(1);
  });

  test('should call dispatchIconError when robotPrimaryColor throws an error', async () => {
    jest.spyOn(colorManager, 'robotPrimaryColor').mockRejectedValue(() => 'err');
    const mockDispatchIconError = jest.fn();
    const root = mount(
      <svg>
        <RobotImageIcon
          robot={makeRobot()}
          footprint={1}
          colorManager={colorManager}
          dispatchIconError={mockDispatchIconError}
          iconPath={iconPath}
          fleetName={fleetName}
        />
      </svg>,
    );
    await act(
      () =>
        new Promise<void>((resolve) => {
          setImmediate(() => {
            root.update();
            resolve();
          });
        }),
    );
    expect(mockDispatchIconError).toBeCalledTimes(1);
  });
});
