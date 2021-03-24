import React from 'react';
import { cleanup, render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { BrowserRouter } from 'react-router-dom';
import FakeAuthenticator from '../../lib/fake-authenticator';
import { LoginBase } from '../../lib/components/login-base';

describe('Login page', () => {
  afterEach(() => {
    cleanup();
  });

  it('renders without crashing', () => {
    const authenticator = new FakeAuthenticator();
    const root = render(
      <BrowserRouter>
        <LoginBase title={'Test'} successRedirectUri={'/test'} authenticator={authenticator} />
      </BrowserRouter>,
    );
    root.unmount();
  });

  it('shows the title correctly', () => {
    const authenticator = new FakeAuthenticator();
    const root = render(
      <BrowserRouter>
        <LoginBase title={'Test'} successRedirectUri={'/test'} authenticator={authenticator} />
      </BrowserRouter>,
    );
    expect(screen.getByText('Test')).toBeTruthy();
    root.unmount();
  });

  it('performs login when login button is clicked', () => {
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
    root.unmount();
  });
});
