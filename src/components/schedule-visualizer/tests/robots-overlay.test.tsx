import React from 'react';
import { mount } from 'enzyme';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import RobotsOverlay from '../robots-overlay';
import ColorManager from '../colors';
import Robot from '../robot';
import fakeFleets from '../../../mock/data/fleets';
import getBuildingMap from '../../../mock/data/building-map';
import { createMuiTheme } from '@material-ui/core';

describe('Robots Overlay', async () => {
  const buildingMap = await getBuildingMap();
  const fleets = fakeFleets();
  const robots = fleets[0].robots;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const colorManager = new ColorManager();
  // TextEncoder is not available in node
  colorManager.robotColor = jest.fn(async () => 'black');
  colorManager.robotColorFromCache = jest.fn(() => 'black');
  let conflictRobotNames: string[][] = [];
  const theme = createMuiTheme();

  test('Render robots correctly', async () => {
    const wrapper = mount(
      <LMap>
        <RobotsOverlay
          fleets={fleets}
          bounds={bounds}
          colorManager={colorManager}
          conflictRobotNames={conflictRobotNames}
          currentFloorName={buildingMap.levels[0].name}
        />
      </LMap>,
    );

    expect(wrapper.find(Robot).exists()).toBeTruthy();
    expect(wrapper.find(Robot).length).toBe(robots.length);

    wrapper.unmount();
  });

  test('Red shadow appears when robots are in trajectory conflict', async () => {
    conflictRobotNames = [[fakeFleets()[0].robots[0].name]];
    const wrapper = mount(
      <LMap>
        <RobotsOverlay
          fleets={fleets}
          bounds={bounds}
          colorManager={colorManager}
          conflictRobotNames={conflictRobotNames}
          currentFloorName={buildingMap.levels[0].name}
        />
      </LMap>,
    );

    expect(
      wrapper.containsMatchingElement(<feDropShadow floodColor={theme.palette.error.main} />),
    ).toBeTruthy();

    wrapper.unmount();
  });
});
