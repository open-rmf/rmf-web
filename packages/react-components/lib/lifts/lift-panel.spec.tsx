import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LiftPanel } from './lift-panel';
import { testLifts, testLiftStates } from './test-utils.spec';

function renderLiftPanel() {
  return render(<LiftPanel lifts={testLifts} liftStates={testLiftStates} />);
}

describe('Lift Panel', () => {
  let root: ReturnType<typeof renderLiftPanel>;
  let mockRequestSubmit: jasmine.Spy<jasmine.Func>;

  beforeEach(() => {
    mockRequestSubmit = jasmine.createSpy();
    root = render(
      <LiftPanel
        lifts={testLifts}
        liftStates={testLiftStates}
        onRequestSubmit={mockRequestSubmit}
      />,
    );
  });

  it('should open up form dialog when Request Form button is clicked', () => {
    userEvent.click(root.getAllByRole('button', { name: /Request Form/i })[0]);
    expect(root.getByPlaceholderText('Pick a Destination')).toBeTruthy();
  });

  it('layout view should change when view mode button is clicked', () => {
    userEvent.click(root.getByLabelText('view-mode'));
    expect(root.getByLabelText('lift-table')).toBeTruthy();
  });
});
