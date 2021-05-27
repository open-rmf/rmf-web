import React from 'react';
import { ProgressBar } from '../../lib/robots/progressbar';
import { render, screen } from '@testing-library/react';

describe('Progress Bar', () => {
  it('renders the progress bar', () => {
    render(<ProgressBar value={100} />);
    expect(screen.getByRole('progressbar')).toBeTruthy();
    expect(screen.getByText('100%')).toBeTruthy();
  });
});
