import React from 'react';
import { mount } from 'enzyme';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import RobotsOverlay from '../robots-overlay';
import ColorManager from '../colors';
import FleetManager from '../../../fleet-manager';
import Robot from '../robot';

test('Render robots correctly', () => {
  let clicked = false;
  const handleClick = () => {
    clicked = true;
  };
  const robots = new FleetManager()
    .fleets()
    .flatMap(x => x.robots.filter(r => r.location.level_name === 'L1'));
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const colorManager = new ColorManager();
  const wrapper = mount(
    <LMap>
      <RobotsOverlay
        bounds={bounds}
        colorManager={colorManager}
        robots={robots}
        onRobotClick={handleClick}
      />
    </LMap>,
  );
  expect(wrapper.find(Robot).exists()).toBeTruthy;
  expect(wrapper.find(Robot).length).toBe(robots.length);

  wrapper.unmount();
});
