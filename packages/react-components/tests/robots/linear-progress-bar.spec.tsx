import React from 'react';
import { LinearProgressBar } from '../../lib/robots/linear-progress-bar';
import { render, screen } from '@testing-library/react';

describe('LinearProgress Bar', () => {
  it('renders the progress bar', () => {
    render(<LinearProgressBar value={100} />);
    expect(screen.getByRole('progressbar')).toBeTruthy();
    expect(screen.getByText('100%')).toBeTruthy();
  });
});
