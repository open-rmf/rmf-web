import React from 'react';
import { render, cleanup } from '@testing-library/react';
import { CircularProgressBar } from '../../lib/robots/circular-progress-bar';

describe('Circular Progress Bar', () => {
  it('smoke test', () => {
    render(<CircularProgressBar progress={60} strokeColor="green" />);
    cleanup();
  });
});
