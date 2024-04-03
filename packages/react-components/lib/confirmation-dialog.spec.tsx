import { render, fireEvent } from '@testing-library/react';
import React from 'react';
import { ConfirmationDialog } from './confirmation-dialog';

describe('ConfirmDialogActions', () => {
  it('calls onClose when cancel button is clicked', () => {
    const onClose = jest.fn();
    const root = render(<ConfirmationDialog open={true} onClose={onClose} />);
    fireEvent.click(root.getByText('Cancel'));
    expect(onClose).toHaveBeenCalled();
  });

  it('calls onSubmit when form is submitted', () => {
    const onSubmit = jest.fn();
    const root = render(<ConfirmationDialog open={true} onSubmit={onSubmit} />);
    fireEvent.click(root.getByText('OK'));
    expect(onSubmit).toHaveBeenCalled();
  });
});
