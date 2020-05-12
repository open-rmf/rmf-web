import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import getBuildingMap from '../../mock/data/building-map';
import Door from './door';

const mount = createMount();

it('Trigger click event', async () => {
  const buildingMap = await getBuildingMap();
  const doors = buildingMap.levels.flatMap(x => x.doors);
  const door = doors[0];
  let clicked = false;

  const wrapper = mount(
    <svg>
      <Door door={door} onClick={() => (clicked = true)} />
    </svg>,
  );

  wrapper
    .find(Door)
    .at(0)
    .simulate('click');
  expect(clicked).toBe(true);

  wrapper.unmount();
});
