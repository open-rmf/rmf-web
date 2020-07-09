import React from 'react';
import { mount } from 'enzyme';
import officeMap from '../../../mock/data/building-map-office';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import PlacesOverlay from '../places-overlay';
import Place from '../place';

test('Render places correctly', () => {
  let clicked = false;
  const handleClick = () => {
    clicked = true;
  };
  const places = officeMap.places;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const wrapper = mount(
    <LMap>
      <PlacesOverlay bounds={bounds} places={places} onPlaceClick={handleClick} />
    </LMap>,
  );
  expect(wrapper.find(Place).exists()).toBeTruthy();
  expect(wrapper.find(Place).length).toBe(officeMap.places.length);

  wrapper.unmount();
});
