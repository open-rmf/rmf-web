import React from 'react';
import { render } from '../../tests/test-utils';
import { AdminDrawer } from '../drawer';

describe('AdminDrawer', () => {
  it('smoke test', () => {
    render(<AdminDrawer />);
  });
});
