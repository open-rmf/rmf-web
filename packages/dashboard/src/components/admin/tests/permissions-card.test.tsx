import { render, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { getActionText, RmfAction } from '../../permissions';
import { PermissionsCard } from '../permissions-card';

describe('PermissionsCard', () => {
  it('renders permissions', async () => {
    const root = render(
      <PermissionsCard
        getPermissions={() => [{ action: RmfAction.TaskRead, authz_grp: 'test_group' }]}
      />,
    );
    await expect(
      waitFor(() => root.getByText(getActionText(RmfAction.TaskRead))),
    ).resolves.not.toThrow();
    await expect(waitFor(() => root.getByText('test_group'))).resolves.not.toThrow();
  });

  it('opens add permission dialog when button is clicked', () => {
    const root = render(<PermissionsCard />);
    userEvent.click(root.getByLabelText('add permission'));
    expect(() => root.getByText('Add Permission')).not.toThrow();
  });

  it('calls removePermission when button is clicked', async () => {
    const removePermission = jest.fn();
    const root = render(
      <PermissionsCard
        getPermissions={() => [{ action: RmfAction.TaskRead, authz_grp: 'test_group' }]}
        removePermission={removePermission}
      />,
    );
    await waitFor(() => root.getByText('Remove'));
    userEvent.click(root.getByText('Remove'));
    await waitFor(() => root.getByLabelText('loading'));
    expect(removePermission).toHaveBeenCalled();
    expect(removePermission.mock.calls[0][0].action).toBe(RmfAction.TaskRead);
    expect(removePermission.mock.calls[0][0].authz_grp).toBe('test_group');
  });
});
