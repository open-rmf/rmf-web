import { createMount } from '@material-ui/core/test-utils';
import { ReactWrapper } from 'enzyme';
import React from 'react';
import ReactTestUtils from 'react-dom/test-utils';
import Dashboard from '../dashboard';

const mount = createMount();

// react-leaflet doesn't work well in jsdom.
jest.mock('./../schedule-visualizer', () => () => null);

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();

  let wrapper: ReactWrapper | undefined;
  await ReactTestUtils.act(async () => {
    wrapper = mount(<Dashboard />);
  });
  wrapper?.unmount();

  (URL.createObjectURL as jest.Mock).mockReset();
});

it('sets showOmniPanel boolean to false', async () => {
  let wrapper: ReactWrapper | undefined;

  await ReactTestUtils.act(async () => {
    wrapper = mount(<Dashboard />);
  });
  wrapper?.find('button').find('#toggle-omnipanel-btn').simulate('click');
  expect(wrapper?.find('Transition').at(1).prop('appear')).toEqual(false);
});
