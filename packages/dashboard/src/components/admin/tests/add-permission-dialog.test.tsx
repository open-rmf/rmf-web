import { render, screen, waitForElementToBeRemoved } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { RmfAction } from '../../permissions';
import { AddPermissionDialog } from '../add-permission-dialog';

describe('AddPermissionDialog', () => {
  it('calls savePermission when form is submitted', async () => {
    const savePermission = jest.fn();
    const root = render(<AddPermissionDialog open={true} savePermission={savePermission} />);
    const user = userEvent.setup();
    await user.click(root.getByLabelText('Action'));
    await user.click(root.getByText('Task Read'));
    await user.type(root.getByLabelText('Authorization Group'), 'test group');
    await user.click(root.getByText('Save'));
    expect(savePermission).toHaveBeenCalled();
    expect(savePermission.mock.calls[0][0].action).toBe(RmfAction.TaskRead);
    expect(savePermission.mock.calls[0][0].authz_grp).toBe('test group');
  });
});
