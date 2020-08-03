import React from 'react';
import { mount } from 'enzyme';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import RobotsOverlay from '../robots-overlay';
import ColorManager from '../colors';
import Robot from '../robot';
import fakeFleets from '../../../mock/data/fleets';

describe('Robots Overlay', () => {
  test('Render robots correctly', async () => {
    const robots = fakeFleets()[0].robots;
    const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
    const colorManager = new ColorManager();
    // TextEncoder is not available in node
    colorManager.robotColor = jest.fn(async () => 'black');
    colorManager.robotColorFromCache = jest.fn(() => 'black');
    const conflictRobotNames: string[] = [];

    const wrapper = mount(
      <LMap>
        <RobotsOverlay bounds={bounds} colorManager={colorManager} robots={robots} conflictRobotNames={conflictRobotNames} />
      </LMap>,
    );

    expect(wrapper.find(Robot).exists()).toBeTruthy();
    expect(wrapper.find(Robot).length).toBe(robots.length);

    wrapper.unmount();
  });

  test('Robot overlay increases in size when it is in trajectory conflict', async () => {
    const robots = fakeFleets()[0].robots;
    const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
    const colorManager = new ColorManager();
    // TextEncoder is not available in node
    colorManager.robotColor = jest.fn(async () => 'black');
    colorManager.robotColorFromCache = jest.fn(() => 'black');
    const conflictRobotNames = [fakeFleets()[0].robots[0].name];

    const wrapper = mount(
      <LMap>
        <RobotsOverlay bounds={bounds} colorManager={colorManager} robots={robots} conflictRobotNames={conflictRobotNames} />
      </LMap>,
    );
    expect(wrapper.containsMatchingElement(<circle r={0.75} />)).toBeTruthy();
    wrapper.unmount();
  })

});