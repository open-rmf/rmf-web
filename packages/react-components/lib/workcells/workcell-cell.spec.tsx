import { render } from '@testing-library/react';
import React from 'react';
import { WorkcellCell } from './workcell-cell';

describe('Workcell Panel', () => {
  it('smoke test', () => {
    expect(() => render(<WorkcellCell guid="test_guid" />)).not.toThrow();
  });
});
