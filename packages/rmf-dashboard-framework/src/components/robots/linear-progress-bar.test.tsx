import { render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';

import { LinearProgressBar } from './linear-progress-bar';

describe('LinearProgress Bar', () => {
  it('renders the progress bar', () => {
    render(<LinearProgressBar value={100} />);
    expect(screen.getByRole('progressbar')).toBeTruthy();
    expect(screen.getByText('100%')).toBeTruthy();
  });
});
