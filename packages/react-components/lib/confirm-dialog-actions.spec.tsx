import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ConfirmDialogActions } from './confirm-dialog-actions';

describe('ConfirmDialogActions', () => {
  it('shows loading when performing confirmation action', () => {
    const root = render(
      <ConfirmDialogActions confirmAction={() => new Promise((res) => setTimeout(res, 500))} />,
    );
    userEvent.click(root.getByText('OK'));
    expect(() => root.getByLabelText('loading')).not.toThrow();
  });
});
