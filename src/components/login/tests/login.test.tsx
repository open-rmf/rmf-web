import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import Login from '../login';

const mount = createMount();

const buildWrapper = () => {
  const wrapper = mount(<Login />);
  return wrapper;
};

describe('Form validation', () => {
  test('Sign in button is not available if user and password are empty ', () => {
    const wrapper = buildWrapper();
    expect(wrapper.findWhere(x => x.name() === 'username' && x.props().value === '')).toBeTruthy();
    expect(wrapper.findWhere(x => x.name() === 'password' && x.props().value === '')).toBeTruthy();
    expect(wrapper.find('button').is('[disabled]')).toBeTruthy();
  });
});
