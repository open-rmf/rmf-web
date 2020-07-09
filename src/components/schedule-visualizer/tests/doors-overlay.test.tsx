import DoorsOverlay from '../doors-overlay';
import React from 'react';
import { mount } from 'enzyme';
import L from 'leaflet';
import { Map as LMap } from 'react-leaflet';
import DoubleSlideDoor from '../door/door-double-slide';
import SingleSlideDoor from '../door/door-single-slide';

// 1 DOOR_TYPE_SINGLE_SLIDING
// 1 DOOR_TYPE_DOUBLE_SLIDING
const doors = [
  {
    name: 'main_door',
    v1_x: 8.2,
    v1_y: -5.5,
    v2_x: 7.85,
    v2_y: -6.2,
    door_type: 2,
    motion_range: -1.571,
    motion_direction: 1,
  },
  {
    name: 'exit_door',
    v1_x: 12.2,
    v1_y: -2.7,
    v2_x: 14.1,
    v2_y: -2.7,
    door_type: 1,
    motion_range: -1.571,
    motion_direction: 1,
  },
];

test('Render doors correctly', () => {
  let clicked = false;
  const handleClick = () => {
    clicked = true;
  };
  const testDoors = doors;
  const bounds = new L.LatLngBounds([0, 25.7], [-14, 0]);
  const wrapper = mount(
    <LMap>
      <DoorsOverlay bounds={bounds} doors={testDoors} onDoorClick={handleClick} />
    </LMap>,
  );
  // 1 normal single sliding + 2 Door slides of the double sliding
  expect(wrapper.find(SingleSlideDoor).length).toBe(3);
  expect(wrapper.find(DoubleSlideDoor).exists()).toBeTruthy();

  wrapper.unmount();
});
