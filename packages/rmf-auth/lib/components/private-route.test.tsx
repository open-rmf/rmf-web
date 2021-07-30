import { render } from '@testing-library/react';
import { createMemoryHistory, MemoryHistory } from 'history';
import React from 'react';
import { Router } from 'react-router-dom';
import { PrivateRoute } from './private-route';

describe('PrivateRoute', () => {
  let history: MemoryHistory;

  beforeEach(() => {
    history = createMemoryHistory();
    history.location.pathname = '/private';
  });

  test('renders unauthorizedComponent when unauthenticated', () => {
    const root = render(
      <Router history={history}>
        <PrivateRoute path="/private" exact unauthorizedComponent="test" user={null} />
      </Router>,
    );
    expect(() => root.getByText('test')).not.toThrow();
  });

  test('renders children when authenticated', () => {
    const root = render(
      <Router history={history}>
        <PrivateRoute path="/private" exact user="test" unauthorizedComponent="unauthorized">
          authorized
        </PrivateRoute>
      </Router>,
    );
    expect(() => root.getByText('authorized')).not.toThrow();
  });
});
