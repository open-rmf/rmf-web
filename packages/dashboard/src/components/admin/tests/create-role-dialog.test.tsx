import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';

import { CreateRoleDialog } from '../create-role-dialog';

describe('CreateRoleDialog', () => {
  it('calls createRole when form is submitted', async () => {
    const createRole = vi.fn();
    const root = render(<CreateRoleDialog open={true} createRole={createRole} />);
    await userEvent.type(root.getByLabelText('Role'), 'role');
    await userEvent.click(root.getByText('Create'));
    expect(createRole).toHaveBeenCalled();
    expect(createRole.mock.calls[0][0]).toBe('role');
  });
});
