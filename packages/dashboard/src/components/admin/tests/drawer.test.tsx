import { describe, it } from 'vitest';
import { render } from '../../tests/test-utils';
import { AdminDrawer } from '../drawer';

describe('AdminDrawer', () => {
  it('smoke test', () => {
    render(<AdminDrawer />);
  });
});
