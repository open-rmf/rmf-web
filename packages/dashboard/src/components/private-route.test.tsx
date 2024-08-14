import { render } from '@testing-library/react';
import { createMemoryHistory, MemoryHistory } from 'history';
import { Router } from 'react-router-dom';
import { beforeEach, describe, expect, it } from 'vitest';

import { PrivateRoute } from './private-route';

describe('PrivateRoute', () => {
  let history: MemoryHistory;

  beforeEach(() => {
    history = createMemoryHistory();
    history.push('/private');
  });

  it('renders unauthorizedComponent when unauthenticated', () => {
    const root = render(
      <Router location={history.location} navigator={history}>
        <PrivateRoute path="/private" unauthorizedComponent="test" user={null} />
      </Router>,
    );
    expect(() => root.getByText('test')).not.toThrow();
  });

  it('renders children when authenticated', () => {
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
