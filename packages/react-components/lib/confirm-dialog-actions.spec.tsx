import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ConfirmDialogActions } from './confirm-dialog-actions';

describe('ConfirmDialogActions', () => {
  it('calls onCancelClick when cancel button is clicked', () => {
    const onCancelClick = jasmine.createSpy();
    const root = render(<ConfirmDialogActions onCancelClick={onCancelClick} />);
    userEvent.click(root.getByText('Cancel'));
    expect(onCancelClick).toHaveBeenCalled();
  });

  it('calls onConfirmClick when confirm button is clicked', () => {
    const onConfirmClick = jasmine.createSpy();
    const root = render(<ConfirmDialogActions onConfirmClick={onConfirmClick} />);
    userEvent.click(root.getByText('OK'));
    expect(onConfirmClick).toHaveBeenCalled();
  });
});
