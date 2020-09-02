import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import getBuildingMap from '../../../mock/data/building-map';
import Waypoint from '../waypoint';
import toJson from 'enzyme-to-json';

const mount = createMount();

it('Renders correctly', async () => {
  const buildingMap = await getBuildingMap();
  const nav_graphs = buildingMap.levels.flatMap(x => x.nav_graphs);
  const waypoints = nav_graphs[0].vertices;
  const waypoint = waypoints[0];

  const wrapper = mount(
    <svg>
      <Waypoint waypoint={waypoint} size={1} />
    </svg>,
  );

  expect(
    wrapper
      .find('rect')
      .at(0)
      .exists(),
  ).toBeTruthy();

  expect(
    wrapper
      .find('text')
      .at(0)
      .exists(),
  ).toBeTruthy();

  expect(toJson(wrapper)).toMatchSnapshot();

  wrapper.unmount();
});
