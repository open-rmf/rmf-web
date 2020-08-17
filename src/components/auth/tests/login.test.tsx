import { createMount } from '@material-ui/core/test-utils';
import { Router } from '@material-ui/icons';
import React from 'react';
import { Redirect } from 'react-router';
import appConfig from '../../../app-config';
import Login from '../login';

const mount = createMount();

const buildWrapper = () => {
  const wrapper = mount(
    <Router>
      <Login />
    </Router>,
  );
  return wrapper;
};

describe('Form validation', () => {
  test('redirects to dashboard when returning from oauth', async () => {
    window.history.replaceState(window.history.state, '', appConfig.authRedirectUri);
    const wrapper = buildWrapper();
    expect(wrapper.find(Redirect)).toBeTruthy();
  });
});
