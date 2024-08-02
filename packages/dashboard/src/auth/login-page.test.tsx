import { render } from '@testing-library/react';
import { it } from 'vitest';

import { LoginPage } from './login-page';

it('smoke test', () => {
  render(<LoginPage title="test" logo="" />);
});
