import { mount } from 'enzyme';
import L from 'leaflet';
import React from 'react';
import { LiftMarker } from 'react-components';
import { Map as LMap } from 'react-leaflet';
import officeMap from '../../../mock/data/building-map-office';
import LiftsOverlay from '../lift-overlay';

test('Render lifts correctly', () => {
  const lifts = officeMap.lifts;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const wrapper = mount(
    <LMap>
      <LiftsOverlay currentFloor={'L1'} bounds={bounds} lifts={lifts} />
    </LMap>,
  );
  expect(wrapper.find(LiftMarker).length).toBe(officeMap.lifts.length);

  wrapper.unmount();
});
