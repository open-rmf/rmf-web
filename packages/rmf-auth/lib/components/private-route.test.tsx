import { render } from '@testing-library/react';
import { createMemoryHistory, MemoryHistory } from 'history';

import React from 'react';
import { Router } from 'react-router-dom';
import { PrivateRoute } from './private-route';

describe('PrivateRoute', () => {
  let history: MemoryHistory;

  beforeEach(() => {
    history = createMemoryHistory();
    history.push('/private');
  });

  test('renders unauthorizedComponent when unauthenticated', () => {
    const root = render(
      <Router location={history.location} navigator={history}>
        <PrivateRoute path="/private" unauthorizedComponent="test" user={null} />
      </Router>,
    );
    expect(() => root.getByText('test')).not.toThrow();
  });

  test('renders children when authenticated', () => {
    const root = render(
      <Router location={history.location} navigator={history}>
        <PrivateRoute path="/private" user="test" unauthorizedComponent="unauthorized">
          authorized
        </PrivateRoute>
      </Router>,
    );
    expect(() => root.getByText('authorized')).not.toThrow();
  });
});
