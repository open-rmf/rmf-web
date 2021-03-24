import { createMemoryHistory, MemoryHistory } from 'history';
import React from 'react';
import { cleanup, render, waitFor } from '@testing-library/react';
import { BrowserRouter, Router, Switch } from 'react-router-dom';
import PrivateRoute from '../../lib/components/private-route';
import FakeAuthenticator from '../../lib/fake-authenticator';

describe('PrivateRoute', () => {
  let history: MemoryHistory;

  beforeEach(() => {
    history = createMemoryHistory();
    history.location.pathname = '/private';
  });

  afterEach(() => {
    cleanup();
  });

  test('renders correctly', () => {
    const root = render(
      <BrowserRouter>
        <PrivateRoute
          path="/private"
          exact
          loginRoute={'/login'}
          user={{ username: 'test' }}
          unauthorized={<h1>unauthorized</h1>}
        />
      </BrowserRouter>,
    );
    root.unmount();
  });

  test('redirects when unauthenticated', () => {
    const authenticator = new FakeAuthenticator();
    const root = render(
      <Router history={history}>
        <Switch>
          <PrivateRoute
            path="/private"
            exact
            loginRoute={'/login'}
            user={authenticator.user}
            unauthorized={<h1>unauthorized</h1>}
          />
        </Switch>
      </Router>,
    );
    expect(history.location.pathname).toBe('/login');
    root.unmount();
  });

  test('no redirects when authenticated', () => {
    const root = render(
      <Router history={history}>
        <Switch>
          <PrivateRoute
            path="/private"
            exact
            loginRoute={'login'}
            user={{ username: 'test' }}
            unauthorized={<h1>unauthorized</h1>}
          />
        </Switch>
      </Router>,
    );
    expect(history.location.pathname).toBe('/private');
    root.unmount();
  });

  test('shows unauthorized page when noRedirectToLogin is true', async () => {
    const root = render(
      <BrowserRouter>
        <PrivateRoute
          path="/private"
          exact
          noRedirectToLogin
          user={{ username: 'test' }}
          unauthorized={<h1>Unauthorized</h1>}
        />
      </BrowserRouter>,
    );
    waitFor(() => {
      expect(root.queryByText('Unauthorized')).toBeTruthy();
    });
    expect(history.location.pathname).toBe('/private');
  });
});
