import { render } from '@testing-library/react';
import React from 'react';
import { LogManagement } from '../../lib';

it('smoke test', () => {
  render(<LogManagement getLabels={jest.fn()} getLogs={jest.fn()} />);
});
