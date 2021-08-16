import { waitFor } from '@testing-library/react';
import L from 'leaflet';
import React from 'react';
import { RobotMarkerProps } from 'react-components';
import ResourceManager from '../../../managers/resource-manager';
import { RobotResourceManager } from '../../../managers/resource-manager-robots';
import { ResourcesContext } from '../../app-contexts';
import RobotsOverlay from '../robots-overlay';
import getBuildingMap from './building-map';
import fakeFleets from './fleets';
import { render } from './leaflet-fixture';

// need to use fake marker because jsdom doesn't support `getComputedTextLength`
function FakeMarker(props: RobotMarkerProps & { 'data-testid'?: string }) {
  return <div data-testid={props['data-testid']} data-testicon={props.iconPath}></div>;
}

describe('Robots Overlay', () => {
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  let conflictRobotNames: string[][] = [];

  test('Render robots correctly', async () => {
    const buildingMap = await getBuildingMap();
    const fleet = fakeFleets()[0];
    const robots = fleet.robots;
    const root = render(
      <RobotsOverlay
        fleets={[fleet]}
        bounds={bounds}
        conflictRobotNames={conflictRobotNames}
        currentFloorName={buildingMap.levels[0].name}
        MarkerComponent={FakeMarker}
      />,
    );
    await waitFor(() => expect(root.getAllByTestId('robotMarker').length).toBe(robots.length));
    root.unmount();
  });

  test('use image when available', async () => {
    const buildingMap = await getBuildingMap();
    const fleet = fakeFleets()[0];
    const robotResourcesMgr = new RobotResourceManager({});
    robotResourcesMgr.getIconPath = () => Promise.resolve('/test-icon.png');
    const resourceMgr: ResourceManager = { robots: robotResourcesMgr };

    const root = render(
      <ResourcesContext.Provider value={resourceMgr}>
        <RobotsOverlay
          fleets={[fleet]}
          bounds={bounds}
          conflictRobotNames={conflictRobotNames}
          currentFloorName={buildingMap.levels[0].name}
          MarkerComponent={FakeMarker}
        />
      </ResourcesContext.Provider>,
    );
    await expect(
      waitFor(() => {
        const q = root.container.querySelector('[data-testicon="/test-icon.png"]');
        if (!q) {
          throw new Error();
        }
        return q;
      }),
    ).resolves.not.toThrow();
  });
});
