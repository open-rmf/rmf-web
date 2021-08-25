import { render, waitFor } from '@testing-library/react';
import L from 'leaflet';
import React from 'react';
import { getPlaces } from 'react-components';
import { Map as LMap } from 'react-leaflet';
import { PlacesContext } from '../../rmf-app';
import WaypointsOverlay from '../waypoints-overlay';
import getBuildingMap from './building-map';

test('Render waypoints correctly', async () => {
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const buildingMap = await getBuildingMap();
  const currentLevel = buildingMap.levels[0];
  const places = getPlaces(buildingMap);
  const waypoints = Object.values(places);

  const root = render(
    <PlacesContext.Provider value={places}>
      <LMap
        bounds={[
          [0, 0],
          [1, 1],
        ]}
      >
        <WaypointsOverlay bounds={bounds} currentLevel={currentLevel} />
      </LMap>
    </PlacesContext.Provider>,
  );

  await waitFor(() => {
    expect(root.getAllByTestId('waypointMarker').length).toBe(waypoints.length);
  });

  root.unmount();
});
