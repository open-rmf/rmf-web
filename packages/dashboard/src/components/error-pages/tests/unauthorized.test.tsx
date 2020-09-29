import { mount } from 'enzyme';
import React from 'react';
import Unauthorized from '../unauthorized';

describe('Unauthorized', () => {
  test('renders correctly', () => {
    const root = mount(<Unauthorized />);
    expect(root.html()).toMatchSnapshot();
  });
});
