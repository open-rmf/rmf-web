import { createMount } from '@material-ui/core/test-utils';
import { createMemoryHistory, MemoryHistory } from 'history';
import React from 'react';
import { BrowserRouter, Router, Switch } from 'react-router-dom';
import { LOGIN_ROUTE } from '../../../util/url';
import { UserContext } from '../../auth/contexts';
import Unauthorized from '../../error-pages/unauthorized';
import PrivateRoute from '../private-route';

const mount = createMount();

describe('PrivateRoute', () => {
  let history: MemoryHistory;

  beforeEach(() => {
    history = createMemoryHistory();
    history.location.pathname = '/private';
  });

  test('renders correctly', () => {
    const component = mount(
      <BrowserRouter>
        <PrivateRoute path="/private" exact />
      </BrowserRouter>,
    );
    expect(component.html()).toMatchSnapshot();
  });

  test('redirects when unauthenticated', () => {
    mount(
      <Router history={history}>
        <Switch>
          <PrivateRoute path="/private" exact />
        </Switch>
      </Router>,
    );
    expect(history.location.pathname).toBe(LOGIN_ROUTE);
  });

  test('no redirects when authenticated', () => {
    mount(
      <UserContext.Provider value={{ username: 'test' }}>
        <Router history={history}>
          <Switch>
            <PrivateRoute path="/private" exact />
          </Switch>
        </Router>
      </UserContext.Provider>,
    );
    expect(history.location.pathname).toBe('/private');
  });

  test('shows unauthorized page when noRedirectToLogin is true', () => {
    const component = mount(
      <BrowserRouter>
        <PrivateRoute path="/private" exact noRedirectToLogin />
      </BrowserRouter>,
    );
    expect(history.location.pathname).toBe('/private');
    expect(component.find(Unauthorized)).toBeTruthy();
  });
});
