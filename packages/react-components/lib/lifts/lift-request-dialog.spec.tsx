import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LiftRequestDialog } from './lift-request-dialog';
import { requestDoorModes, requestModes } from './lift-utils';
import { makeLift, makeLiftState } from './test-utils.spec';

function renderLiftRequestForm() {
  const mockOnClose = jasmine.createSpy();
  const lift = makeLift();
  const liftState = makeLiftState();
  return render(
    <LiftRequestDialog
      currentLevel={liftState.current_floor}
      availableLevels={lift.levels}
      availableRequestTypes={requestModes}
      availableDoorModes={requestDoorModes}
      showFormDialog={true}
      onClose={mockOnClose}
    />,
  );
}

describe('Lift request form', () => {
  let root: ReturnType<typeof renderLiftRequestForm>;

  beforeEach(() => {
    const mockOnClose = jasmine.createSpy();
    const lift = makeLift();
    const liftState = makeLiftState();
    root = render(
      <LiftRequestDialog
        currentLevel={liftState.current_floor}
        availableLevels={lift.levels}
        availableRequestTypes={requestModes}
        availableDoorModes={requestDoorModes}
        showFormDialog={true}
        onClose={mockOnClose}
      />,
    );
  });

  it('destination is required', () => {
    userEvent.type(root.getByPlaceholderText('Pick a Destination'), '{selectall}{backspace}');
    userEvent.click(root.getByText('Request'));
    expect(root.getByText('Destination cannot be empty')).toBeTruthy();
  });
});
