import { createMemoryHistory, MemoryHistory } from 'history';
import React from 'react';
import { cleanup, render, waitFor, RenderResult } from '@testing-library/react';
import { BrowserRouter, Router, Switch } from 'react-router-dom';
import PrivateRoute from '../lib/components/private-route';
import FakeAuthenticator from '../lib/__mocks__/authenticator';

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
    render(
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
  });

  test('no redirects when authenticated', () => {
    render(
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
  });

  test('shows unauthorized page when noRedirectToLogin is true', async () => {
    let root: RenderResult;
    waitFor(() => {
      root = render(
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
      expect(root.queryByText('Unauthorized')).toBeTruthy();
    });
    expect(history.location.pathname).toBe('/private');
  });
});
