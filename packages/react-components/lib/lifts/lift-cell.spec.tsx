import { render, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LiftCell } from './lift-cell';
import { makeLift } from './test-utils.spec';

describe('Lift Panel', () => {
  const lift = makeLift({ name: 'test_lift' });
  let root: RenderResult;
  let mockRequestSubmit: jasmine.Spy<jasmine.Func>;

  beforeEach(() => {
    mockRequestSubmit = jasmine.createSpy();
    root = render(<LiftCell lift={lift} onRequestSubmit={mockRequestSubmit} />);
  });

  it('should open up form dialog when Request Form button is clicked', () => {
    userEvent.click(root.getAllByRole('button', { name: /Request Form/i })[0]);
    expect(root.getByPlaceholderText('Pick a Destination')).toBeTruthy();
  });
});
