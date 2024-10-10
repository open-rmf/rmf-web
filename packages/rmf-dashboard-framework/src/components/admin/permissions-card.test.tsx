import { fireEvent, render as render_, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { afterAll, beforeAll, describe, expect, it, vi } from 'vitest';

import { AppControllerProvider } from '../../hooks/use-app-controller';
import { getActionText, RmfAction } from '../../services/permissions';
import { makeMockAppController } from '../../utils/test-utils.test';
import { PermissionsCard } from './permissions-card';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

// TODO(AA): To remove after
// https://github.com/testing-library/react-testing-library/issues/1216
// has been resolved.
// Workaround for "Warning: An update to ComponentName inside a test was not
// wrapped in act(...)."
const originalError = console.error;
beforeAll(() => {
  console.error = (...args) => {
    if (/Warning.*not wrapped in act/.test(args[0])) {
      return;
    }
    originalError.call(console, ...args);
  };
});

afterAll(() => {
  console.error = originalError;
});

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

  it('opens add permission dialog when button is clicked', async () => {
    const root = render(<PermissionsCard />);
    await userEvent.click(root.getByLabelText('add permission'));
    expect(() => root.getByText('Add Permission')).not.toThrow();
  });

  it('calls removePermission when button is clicked', async () => {
    const removePermission = vi.fn();
    const { getByText } = render(
      <PermissionsCard
        getPermissions={() => [{ action: RmfAction.TaskRead, authz_grp: 'test_group' }]}
        removePermission={removePermission}
      />,
    );
    await waitFor(() => getByText('Remove'));
    fireEvent.click(getByText('Remove'));
    expect(removePermission).toHaveBeenCalled();
    expect(removePermission.mock.calls[0][0].action).toBe(RmfAction.TaskRead);
    expect(removePermission.mock.calls[0][0].authz_grp).toBe('test_group');
  });
});
