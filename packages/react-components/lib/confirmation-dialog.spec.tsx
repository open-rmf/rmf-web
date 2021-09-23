import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ConfirmationDialog } from './confirmation-dialog';

describe('ConfirmDialogActions', () => {
  it('calls onClose when cancel button is clicked', () => {
    const onClose = jasmine.createSpy();
    const root = render(<ConfirmationDialog open={true} onClose={onClose} />);
    userEvent.click(root.getByText('Cancel'));
    expect(onClose).toHaveBeenCalled();
  });

  it('calls onSubmit when form is submitted', () => {
    const onSubmit = jasmine.createSpy();
    const root = render(<ConfirmationDialog open={true} onSubmit={onSubmit} />);
    userEvent.click(root.getByText('OK'));
    expect(onSubmit).toHaveBeenCalled();
  });

  it('should not show dialog actions section when showDialogActions is false', () => {
    const onSubmit = jasmine.createSpy();
    const root = render(
      <ConfirmationDialog open={true} onSubmit={onSubmit} showDialogActions={false} />,
    );
    expect(root.queryByText('Cancel')).toBeNull();
    expect(root.queryByText('OK')).toBeNull();
  });

  it('should show dialog actions section when showDialogActions is true', () => {
    const onSubmit = jasmine.createSpy();
    const root = render(
      <ConfirmationDialog open={true} onSubmit={onSubmit} showDialogActions={true} />,
    );
    expect(root.queryByText('Cancel')).toBeTruthy();
    expect(root.queryByText('OK')).toBeTruthy();
  });
});
