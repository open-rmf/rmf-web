import { render, fireEvent, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LiftRequestDialog } from './lift-request-dialog';
import { requestDoorModes, requestModes } from './lift-utils';
import { makeLift, makeLiftState } from './test-utils.spec';

describe('Lift request form', () => {
  it('destination is required', () => {
    const mockOnClose = jasmine.createSpy();
    const lift = makeLift();
    const liftState = makeLiftState();
    const root = render(
      <LiftRequestDialog
        currentLevel={liftState.current_floor}
        availableLevels={lift.levels}
        availableRequestTypes={requestModes}
        availableDoorModes={requestDoorModes}
        showFormDialog={true}
        onClose={mockOnClose}
      />,
    );
    const inputEl = root.getByLabelText('Pick a Destination') as HTMLInputElement;
    const user = userEvent.setup();
    user.clear(inputEl);
    expect(inputEl.value).toBe('');
    fireEvent.click(screen.getByText('Request'));
    expect(root.getByText('Destination cannot be empty')).toBeTruthy();
  });
});
