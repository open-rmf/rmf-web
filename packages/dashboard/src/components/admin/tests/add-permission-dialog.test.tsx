import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { RmfAction } from '../../permissions';
import { AddPermissionDialog } from '../add-permission-dialog';

describe('AddPermissionDialog', () => {
  it('calls savePermission when form is submitted', async () => {
    const savePermission = jest.fn();
    const root = render(<AddPermissionDialog open={true} savePermission={savePermission} />);
    await userEvent.click(root.getByLabelText('Action'));
    await userEvent.click(root.getByText('Task Read'));
    await userEvent.type(root.getByLabelText('Authorization Group'), 'test group');
    await userEvent.click(root.getByText('Save'));
    expect(savePermission).toHaveBeenCalled();
    expect(savePermission.mock.calls[0][0].action).toBe(RmfAction.TaskRead);
    expect(savePermission.mock.calls[0][0].authz_grp).toBe('test group');
  });
});
