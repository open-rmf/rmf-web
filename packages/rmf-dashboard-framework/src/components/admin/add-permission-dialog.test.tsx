import { render as render_ } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';

import { AppControllerProvider } from '../../hooks/use-app-controller';
import { RmfAction } from '../../services/permissions';
import { makeMockAppController } from '../../utils/test-utils.test';
import { AddPermissionDialog } from './add-permission-dialog';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('AddPermissionDialog', () => {
  it('calls savePermission when form is submitted', async () => {
    const savePermission = vi.fn();
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
