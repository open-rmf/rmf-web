import { mount } from 'enzyme';
import ImageIcon from '../image-icon';
import React from 'react';

test('Renders correctly', () => {
  const container = mount(
    <svg>
      <ImageIcon iconPath={'test'} height={1} width={1} footprint={0.5} />
    </svg>,
  );
  expect(container).toMatchSnapshot();
});
