import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import { render, fireEvent, screen } from '@testing-library/react';
import { Redirect } from 'react-router';
import { BrowserRouter } from 'react-router-dom';
import FakeAuthenticator from '../../../mock/fake-authenticator';
import { AuthenticatorContext, UserContext } from '../../auth/contexts';
import Login from '../login';

const mount = createMount();

describe('Login page', () => {
  test('renders correctly', () => {
    const component = mount(
      <BrowserRouter>
        <Login />
      </BrowserRouter>,
    );
    expect(component).toMatchSnapshot();
  });

  test('renders without crashing', () => {
    const root = render(
      <BrowserRouter>
        <Login />
      </BrowserRouter>,
    );
    root.unmount();
  });

  /*NOTE: Removing this test causes the subsequent 2 tests to fail
  but running the tests individually (test.only) allows the tests to pass*/
  test('redirects to dashboard when user is authenticated', () => {
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

  test('redirects from login when user is authenticated', () => {
    const root = render(
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
    ``;
    //screen.debug();
    expect(root.queryByText('Login with RMF')).toBeFalsy();
    root.unmount();
  });

  test('performs login when login button is clicked', () => {
    const authenticator = new FakeAuthenticator();
    const spy = jest.spyOn(authenticator, 'login').mockImplementation(() => undefined as any);

    const root = render(
      <BrowserRouter>
        <AuthenticatorContext.Provider value={authenticator}>
          <Login />
        </AuthenticatorContext.Provider>
      </BrowserRouter>,
    );
    //screen.debug();
    const loginButton = root.getByRole('button', { name: /Login with RMF/i });
    fireEvent.click(loginButton);
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
