import { render as render_, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { MemoryRouter } from 'react-router';
import { describe, expect, it, vi } from 'vitest';

import { AppControllerProvider } from '../../hooks/use-app-controller';
import { makeMockAppController } from '../../utils/test-utils.test';
import { UserListCard } from './user-list-card';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('UserListCard', () => {
  it('opens delete dialog when button is clicked', async () => {
    const root = render(
      <MemoryRouter>
        <UserListCard searchUsers={() => ['user1']} />
      </MemoryRouter>,
    );
    await waitFor(() => root.getByText('Delete'));
    await userEvent.click(root.getByText('Delete'));
    expect(root.findByText('Confirm Delete')).resolves.toBeTruthy();
  });

  it('opens create user dialog when button is clicked', async () => {
    const root = render(
      <MemoryRouter>
        <UserListCard />
      </MemoryRouter>,
    );
    await userEvent.click(root.getByLabelText('create user'));
    expect(root.findByText('Create User')).resolves.toBeTruthy();
  });

  it('calls searchUser when search box changes', async () => {
    const searchUsers = vi.fn(() => []);
    const root = render(
      <MemoryRouter>
        <UserListCard searchUsers={searchUsers} />
      </MemoryRouter>,
    );
    await userEvent.type(root.getByLabelText('Search Users'), 'test');
    await waitFor(() => expect(searchUsers).toHaveBeenCalledTimes(2), { timeout: 1000 });
    expect((searchUsers.mock.calls[1] as any[])[0]).toBe('test');
  });
});
