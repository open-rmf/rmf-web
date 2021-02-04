import React from 'react';
import { render } from '@testing-library/react';
import Unauthorized from '../unauthorized';

describe('Unauthorized', () => {
  test('renders correctly', () => {
    const root = render(<Unauthorized />);
    root.unmount();
  });
});
