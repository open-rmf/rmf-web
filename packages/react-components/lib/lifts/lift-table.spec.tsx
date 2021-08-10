import { render } from '@testing-library/react';
import React from 'react';

import { LiftTable } from './lift-table';
import { testLifts, testLiftStates } from './test-utils.spec';

describe('lift table', () => {
  it('should render properly', () => {
    render(<LiftTable lifts={testLifts} liftStates={testLiftStates} />);
  });
});
