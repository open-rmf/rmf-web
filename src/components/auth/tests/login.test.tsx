import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import { Redirect } from 'react-router';
import { BrowserRouter } from 'react-router-dom';
import { UserContext } from '../../../app-contexts';
import Login from '../login';

const mount = createMount();

describe('Login page', () => {
  test('renders correctly', () => {
    const component = mount(
      <BrowserRouter>
        <Login />
      </BrowserRouter>,
    );
    expect(component.html()).toMatchSnapshot();
  });

  test('redirects to dashboard when user is authenticated', async () => {
    window.history.replaceState(window.history.state, '');
    const wrapper = mount(
      <BrowserRouter>
        <UserContext.Provider
          value={{
            username: 'test',
          }}
        >
          <Login />
        </UserContext.Provider>
      </BrowserRouter>,
    );
    expect(wrapper.find(Redirect)).toBeTruthy();
  });
});
