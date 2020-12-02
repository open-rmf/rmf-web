import { render } from '@testing-library/react';
import React from 'react';
import { LoginPage } from '../../lib';

test('smoke test', () => {
  render(<LoginPage title="test" logo="" />);
});
