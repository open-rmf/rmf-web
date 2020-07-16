import React from 'react';
import { mount } from 'enzyme';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import RobotsOverlay from '../robots-overlay';
import ColorManager from '../colors';
import Robot from '../robot';
import fakeFleets from '../../../mock/data/fleets';

test('Render robots correctly', () => {
  const robots = fakeFleets()[0].robots;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const colorManager = new ColorManager();
  // TextEncoder is not available in node
  colorManager.robotColor = jest.fn(async () => 'black');
  colorManager.robotColorFromCache = jest.fn(() => 'black');
  const wrapper = mount(
    <LMap>
      <RobotsOverlay bounds={bounds} colorManager={colorManager} robots={robots} />
    </LMap>,
  );
  expect(wrapper.find(Robot).exists()).toBeTruthy();
  expect(wrapper.find(Robot).length).toBe(robots.length);

  wrapper.unmount();
});
