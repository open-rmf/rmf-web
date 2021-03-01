import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { DoorMarker } from '../../lib';
import { makeDoor } from './test-utils';

it('triggers onClick callback when button is clicked', () => {
  const mockOnClick = jasmine.createSpy();

  const root = render(
    <svg>
      <DoorMarker door={makeDoor()} onClick={mockOnClick} data-testid="marker" />
    </svg>,
  );
  userEvent.click(root.getByTestId('marker'));
  expect(mockOnClick).toHaveBeenCalled();
});
