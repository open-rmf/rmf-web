import { render as render_ } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it, vi } from 'vitest';

import { AppControllerProvider } from '../../hooks/use-app-controller';
import { makeMockAppController } from '../../utils/test-utils.test';
import { CreateUserDialog } from './create-user-dialog';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('CreateUserDialog', () => {
  it('calls createUser when form is submitted', async () => {
    const createUser = vi.fn();
    const root = render(<CreateUserDialog open={true} createUser={createUser} />);
    await userEvent.type(root.getByLabelText('Username'), 'user');
    await userEvent.click(root.getByText('Create'));
    expect(createUser).toHaveBeenCalled();
    expect(createUser.mock.calls[0][0]).toBe('user');
  });
});
