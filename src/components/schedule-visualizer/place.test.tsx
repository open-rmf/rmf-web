import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import getBuildingMap from '../../mock/data/building-map';
import Place from './place';

const mount = createMount();

it('fires click event', async () => {
  const buildingMap = await getBuildingMap();
  const places = buildingMap.levels.flatMap(x => x.places);
  const place = places[0];
  let clicked = false;

  const wrapper = mount(
    <svg>
      <Place place={place} size={1} onClick={() => (clicked = true)} />
    </svg>,
  );

  wrapper
    .find(Place)
    .at(0)
    .simulate('click');
  expect(clicked).toBe(true);

  wrapper.unmount();
});
