import { render, waitForElementToBeRemoved } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { CreateUserDialog } from '../create-user-dialog';

describe('CreateUserDialog', () => {
  it('calls createUser when form is submitted', async () => {
    const createUser = jest.fn();
    const root = render(<CreateUserDialog open={true} createUser={createUser} />);
    const user = userEvent.setup();
    await user.type(root.getByLabelText('Username'), 'user');
    await user.click(root.getByText('Create'));
    expect(createUser).toHaveBeenCalled();
    expect(createUser.mock.calls[0][0]).toBe('user');
  });
});
