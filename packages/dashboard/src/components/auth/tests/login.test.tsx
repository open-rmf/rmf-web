import React from 'react';
import { cleanup, render, fireEvent } from '@testing-library/react';
import { BrowserRouter } from 'react-router-dom';
import FakeAuthenticator from '../../../mock/fake-authenticator';
import { AuthenticatorContext, UserContext } from '../../auth/contexts';
import Login from '../login';

describe('Login page', () => {
  afterEach(() => {
    cleanup();
  });

  test('renders without crashing', () => {
    const root = render(
      <BrowserRouter>
        <Login />
      </BrowserRouter>,
    );
    root.unmount();
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
    const loginButton = root.getByRole('button', { name: /Login with RMF/i });
    fireEvent.click(loginButton);
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
