import { render } from '@testing-library/react';
import React from 'react';
import { ErrorSnackbar } from './error-snackbar';

describe('ErrorSnackbar', () => {
  it('shows error message', () => {
    const root = render(<ErrorSnackbar message="test" open={true} />);
    expect(() => root.getByText('test')).not.toThrow();
  });
});
