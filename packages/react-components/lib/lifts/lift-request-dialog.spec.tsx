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
    render(
      <LiftRequestDialog
        currentLevel={liftState.current_floor}
        availableLevels={lift.levels}
        availableRequestTypes={requestModes}
        availableDoorModes={requestDoorModes}
        showFormDialog={true}
        onClose={mockOnClose}
      />,
    );
    userEvent.type(screen.getByPlaceholderText('Pick a Destination'), '{selectall}{backspace}');
    fireEvent.click(screen.getByText('Request'));
    expect(screen.getByText('Destination cannot be empty')).toBeTruthy();
  });
});
