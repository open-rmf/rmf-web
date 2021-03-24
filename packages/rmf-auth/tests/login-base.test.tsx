import React from 'react';
import { cleanup, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { BrowserRouter } from 'react-router-dom';
import FakeAuthenticator from '../lib/__mocks__/authenticator';
import { LoginBase } from '../lib/components/login-base';

describe('Login page', () => {
  afterEach(() => {
    cleanup();
  });

  test('renders without crashing', () => {
    const authenticator = new FakeAuthenticator();
    const root = render(
      <BrowserRouter>
        <LoginBase title={'Test'} successRedirectUri={'/test'} authenticator={authenticator} />
      </BrowserRouter>,
    );
    root.unmount();
  });

  test('performs login when login button is clicked', () => {
    const authenticator = new FakeAuthenticator({ username: 'fakeUser' });
    const spy = jest.spyOn(authenticator, 'login').mockImplementation(() => undefined as any);

    const root = render(
      <BrowserRouter>
        <LoginBase title={'Test'} successRedirectUri={'/test'} authenticator={authenticator} />
      </BrowserRouter>,
    );
    const loginButton = root.getByRole('button', { name: /Login/i });
    userEvent.click(loginButton);

    expect(spy).toHaveBeenCalledTimes(1);
  });
});
