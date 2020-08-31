import React from 'react';
import { mount } from 'enzyme';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import Waypoint from '../waypoint';
import WaypointsOverlay from '../waypoints-overlay';
import getBuildingMap from '../../../mock/data/building-map';

test('Render waypoints correctly', async () => {
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const buildingMap = await getBuildingMap();
  const currentLevel = buildingMap.levels[0];
  const nav_graphs = buildingMap.levels.flatMap(x => x.nav_graphs);
  const waypoints = nav_graphs[0].vertices;

  const wrapper = mount(
    <LMap>
      <WaypointsOverlay bounds={bounds} currentLevel={currentLevel} />
    </LMap>,
  );

  expect(wrapper.find(Waypoint).exists()).toBeTruthy();
  expect(wrapper.find(Waypoint).length).toBe(waypoints.length);

  wrapper.unmount();
});
