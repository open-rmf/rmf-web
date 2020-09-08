import { mount } from 'enzyme';
import ImageIcon from '../image-icon';
import React from 'react';
import toJson from 'enzyme-to-json';

test('Renders correctly', () => {
  const container = mount(<ImageIcon iconPath={'test'} height={1} width={1} footprint={0.5} />);
  expect(toJson(container)).toMatchSnapshot();
});
