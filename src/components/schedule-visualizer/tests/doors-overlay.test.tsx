import DoorsOverlay from '../doors-overlay';
import React from 'react';
import { mount } from 'enzyme';
import officeMap from '../../../mock/data/building-map-office';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import DoubleSlideDoor from '../door/door-double-slide';
import SingleSlideDoor from '../door/door-single-slide';

test('Render doors correctly', () => {
  // 3 DOOR_TYPE_SINGLE_SLIDING
  // 1 DOOR_TYPE_DOUBLE_SLIDING
  let clicked = false;
  const handleClick = () => {
    clicked = true;
  };
  const doors = officeMap.doors;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const wrapper = mount(
    <LMap>
      <DoorsOverlay bounds={bounds} doors={doors} onDoorClick={handleClick} />
    </LMap>,
  );
  // 3 normal single sliding + 2 Door slides of the double sliding
  expect(wrapper.find(SingleSlideDoor).length).toBe(5);
  expect(wrapper.find(DoubleSlideDoor).exists()).toBeTruthy;

  wrapper.unmount();
});
