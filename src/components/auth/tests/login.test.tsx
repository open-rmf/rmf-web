import { createMount } from '@material-ui/core/test-utils';
import { Router } from '@material-ui/icons';
import React from 'react';
import { Redirect } from 'react-router';
import { UserContext } from '../../../app-contexts';
import Login from '../login';

const mount = createMount();

describe('Form validation', () => {
  test('redirects to dashboard when returning from oauth', async () => {
    window.history.replaceState(window.history.state, '');
    const wrapper = mount(
      <Router>
        <UserContext.Provider
          value={{
            username: 'test',
          }}
        >
          <Login />
        </UserContext.Provider>
      </Router>,
    );
    expect(wrapper.find(Redirect)).toBeTruthy();
  });
});
