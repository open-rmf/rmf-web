import { render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { requestDoorModes, requestModes } from '../../lib/lifts/lift-utils';
import { LiftRequestFormDialog } from './lift-request-form-dialog';
import { makeLift } from './test-utils.spec';

function renderLiftRequestForm() {
  const mockOnClose = jasmine.createSpy();
  return render(
    <LiftRequestFormDialog
      lift={makeLift()}
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
    root = render(
      <LiftRequestFormDialog
        lift={makeLift()}
        availableRequestTypes={requestModes}
        availableDoorModes={requestDoorModes}
        showFormDialog={true}
        onClose={mockOnClose}
      />,
    );
  });

  it('resets form after submitting', () => {
    userEvent.click(root.getByPlaceholderText('Pick a Destination'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('L2'));
    userEvent.click(root.getByPlaceholderText('Pick a Door State'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('Closed'));
    userEvent.click(root.getByPlaceholderText('Pick Request Type'));
    userEvent.click(within(screen.getByRole('listbox')).getByText('Human'));

    userEvent.click(root.getByText('Request'));

    expect(root.getByPlaceholderText('Pick a Destination').getAttribute('value')).toBe('L1');
    expect(root.getByPlaceholderText('Pick a Door State').getAttribute('value')).toBe('Open');
    expect(root.getByPlaceholderText('Pick Request Type').getAttribute('value')).toBe('AGV');
  });

  it('destination is required', () => {
    userEvent.type(root.getByPlaceholderText('Pick a Destination'), '{selectall}{backspace}');
    userEvent.click(root.getByText('Request'));
    expect(root.getByText('Destination cannot be empty')).toBeTruthy();
  });
});
