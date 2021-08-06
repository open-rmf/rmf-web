import { render } from '@testing-library/react';
import React from 'react';

import { DoorTable } from './door-table';
import { doorStates, makeDetailedDoors } from './test-utils.spec';

describe('door table', () => {
  it('should render properly', () => {
    render(<DoorTable doors={makeDetailedDoors()} doorStates={doorStates} />);
  });
});
