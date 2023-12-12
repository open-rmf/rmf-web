import { render, waitForElementToBeRemoved } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { CreateRoleDialog } from '../create-role-dialog';

describe('CreateRoleDialog', () => {
  it('calls createRole when form is submitted', async () => {
    const createRole = jest.fn();
    const root = render(<CreateRoleDialog open={true} createRole={createRole} />);
    const user = userEvent.setup();
    await user.type(root.getByLabelText('Role'), 'role');
    await user.click(root.getByText('Create'));
    expect(createRole).toHaveBeenCalled();
    expect(createRole.mock.calls[0][0]).toBe('role');
  });
});
