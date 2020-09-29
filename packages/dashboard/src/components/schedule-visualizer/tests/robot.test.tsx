import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import getFleets from '../../../mock/data/fleets';
import ColorManager from '../colors';
import Robot from '../robot';

const mount = createMount();

it('fires click event', async () => {
  const fleets = await getFleets();
  const robot = fleets[0].robots[0];
  const colorManager = new ColorManager();
  let clicked = false;

  // TextEncoder is not available in node
  colorManager.robotColor = jest.fn(async () => 'black');
  colorManager.robotColorFromCache = jest.fn(() => 'black');

  const root = mount(
    <svg>
      <Robot
        robot={robot}
        footprint={1}
        colorManager={colorManager}
        fleetName=""
        onClick={() => (clicked = true)}
      />
    </svg>,
  );
  root.find(Robot).at(0).simulate('click');
  expect(clicked).toBe(true);

  root.unmount();
});
