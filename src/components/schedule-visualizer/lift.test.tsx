import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import getBuildingMap from '../../mock/data/building-map';
import Lift from './lift';

const mount = createMount();

it('Trigger click event', async () => {
  const buildingMap = await getBuildingMap();
  const lifts = buildingMap.lifts;
  const lift = lifts[0];
  let clicked = false;

  const wrapper = mount(
    <svg>
      <Lift currentFloor={'L1'} lift={lift} onClick={() => (clicked = true)} />
    </svg>,
  );

  wrapper
    .find(Lift)
    .at(0)
    .simulate('click');
  expect(clicked).toBe(true);

  wrapper.unmount();
});
