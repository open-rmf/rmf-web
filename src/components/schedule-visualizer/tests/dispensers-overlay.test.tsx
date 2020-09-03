import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import { ResourcesContext } from '../../../app-contexts';
import fakeResources from '../../../mock/data/resources';
import ResourceManager from '../../../resource-manager';
import DispensersOverlay from '../dispensers-overlay';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import Dispenser from '../dispenser';

const mount = createMount();

describe('Dispensers Overlay', () => {
  test('Render robots correctly', async () => {
    const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
    const resources = new ResourceManager(fakeResources());

    const wrapper = mount(
      <ResourcesContext.Provider value={resources}>
        <LMap>
          <DispensersOverlay bounds={bounds} currentFloorName="L1" />
        </LMap>
      </ResourcesContext.Provider>,
    );

    expect(wrapper.find(Dispenser).exists()).toBeTruthy();
    expect(wrapper.find(Dispenser).length).toBe(2);
    // Object.keys(resources?.dispensers).length

    wrapper.unmount();
  });
});
