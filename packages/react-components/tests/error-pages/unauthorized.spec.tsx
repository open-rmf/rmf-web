import React from 'react';
import { render } from '@testing-library/react';
import Unauthorized from '../../lib/error-pages/unauthorized';

describe('Unauthorized', () => {
  it('renders correctly', () => {
    const root = render(<Unauthorized />);
    expect(root.queryByText('Unauthorized')).toBeTruthy();
    root.unmount();
  });
});
