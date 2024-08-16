import { render } from '@testing-library/react';
import { MemoryRouter } from 'react-router';
import { describe, it } from 'vitest';

import { AdminDrawer } from './drawer';

describe('AdminDrawer', () => {
  it('smoke test', () => {
    render(
      <MemoryRouter>
        <AdminDrawer />
      </MemoryRouter>,
    );
  });
});
