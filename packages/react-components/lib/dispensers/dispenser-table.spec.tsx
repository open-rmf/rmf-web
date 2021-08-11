import { render } from '@testing-library/react';
import React from 'react';

import { DispenserTable } from './dispenser-table';
import { makeDispenser, makeDispenserState } from './test-utils.spec';

describe('Dispenser table', () => {
  it('should render properly', () => {
    render(
      <DispenserTable
        dispensers={[makeDispenser()]}
        dispenserStates={{ test: makeDispenserState() }}
      />,
    );
  });
});
