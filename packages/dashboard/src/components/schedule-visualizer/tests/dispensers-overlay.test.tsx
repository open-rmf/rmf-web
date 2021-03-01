import L from 'leaflet';
import React from 'react';
import { render, waitFor } from '@testing-library/react';
import { Map as LMap } from 'react-leaflet';
import ResourceManager from '../../../managers/resource-manager';
import fakeResources from '../../../managers/__mocks__/resources';
import { ResourcesContext } from '../../app-contexts';
import DispensersOverlay from '../dispensers-overlay';

describe('Dispensers Overlay', () => {
  test('Render dispensers correctly', async () => {
    const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
    const resources = new ResourceManager(fakeResources());

    const root = render(
      <ResourcesContext.Provider value={resources}>
        <LMap
          bounds={[
            [0, 0],
            [1, 1],
          ]}
        >
          <DispensersOverlay bounds={bounds} currentFloorName="L1" />
        </LMap>
      </ResourcesContext.Provider>,
    );

    await waitFor(() => {
      expect(root.queryAllByTestId('dispenserMarker').length).toBeTruthy();
      expect(root.getAllByTestId('dispenserMarker').length).toBe(
        resources.dispensers?.allValues?.length,
      );
    });

    root.unmount();
  });
});
