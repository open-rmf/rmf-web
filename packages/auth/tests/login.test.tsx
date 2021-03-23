import React from 'react';
import { cleanup, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { BrowserRouter } from 'react-router-dom';
import FakeAuthenticator from '../lib/__mocks__/authenticator';
import { Login } from '../lib/components/login';

describe('Login page', () => {
  afterEach(() => {
    cleanup();
  });

  test('renders without crashing', () => {
    const authenticator = new FakeAuthenticator();
    const root = render(
      <BrowserRouter>
        <Login defaultRoute={'/test'} authenticator={authenticator} />
      </BrowserRouter>,
    );
    root.unmount();
  });

  test('redirects from login when user is authenticated', () => {
    const authenticator = new FakeAuthenticator();

    const root = render(
      <BrowserRouter>
        <Login defaultRoute={'/test'} user={{ username: 'test' }} authenticator={authenticator} />
      </BrowserRouter>,
    );

    expect(root.queryByText('Login')).toBeFalsy();
  });

  test('performs login when login button is clicked', () => {
    const authenticator = new FakeAuthenticator({ username: 'fakeUser' });
    const spy = jest.spyOn(authenticator, 'login').mockImplementation(() => undefined as any);

    const root = render(
      <BrowserRouter>
        <Login defaultRoute={'/test'} authenticator={authenticator} />
      </BrowserRouter>,
    );
    const loginButton = root.getByRole('button', { name: /Login/i });
    userEvent.click(loginButton);

    expect(spy).toHaveBeenCalledTimes(1);
  });
});
