import React from 'react';
import { mount } from 'enzyme';
import officeMap from '../../../mock/data/building-map-office';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import SingleSlideDoor from '../door/door-single-slide';
import LiftsOverlay from '../lift-overlay';
import Lift from '../lift';

test('Render lifts correctly', () => {
  let clicked = false;
  const handleClick = () => {
    clicked = true;
  };
  const lifts = officeMap.lifts;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const wrapper = mount(
    <LMap>
      <LiftsOverlay currentFloor={'L1'} bounds={bounds} lifts={lifts} onLiftClick={handleClick} />
    </LMap>,
  );
  expect(wrapper.find(Lift).exists()).toBeTruthy;
  expect(wrapper.find(Lift).length).toBe(officeMap.lifts.length);
  expect(wrapper.find(SingleSlideDoor).length).toBe(2);

  wrapper.unmount();
});
