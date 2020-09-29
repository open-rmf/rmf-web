import { mount } from 'enzyme';
import DispenserDefaultIcon from '../dispenser-default-icon';
import React from 'react';
import toJson from 'enzyme-to-json';

test('Renders correctly', () => {
  const container = mount(<DispenserDefaultIcon footprint={0.5} />);
  expect(container.find('path').exists()).toBeTruthy();
  expect(toJson(container)).toMatchSnapshot();
});
