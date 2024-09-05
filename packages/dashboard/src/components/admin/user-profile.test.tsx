import { render as render_, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { describe, expect, it, vi } from 'vitest';

import { AppControllerProvider } from '../../hooks/use-app-controller';
import { makeMockAppController } from '../../utils/test-utils.test';
import { UserProfileCard } from './user-profile';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('UserProfileCard', () => {
  it('renders username', () => {
    const root = render(
      <UserProfileCard user={{ username: 'test', is_admin: false, roles: [] }} />,
    );
    root.getByText('test');
  });

  it('renders admin', () => {
    const root = render(<UserProfileCard user={{ username: 'test', is_admin: true, roles: [] }} />);
    root.getByText('Admin');
  });

  it('calls makeAdmin when checkbox is clicked', async () => {
    const makeAdmin = vi.fn();
    const root = render(
      <UserProfileCard
        user={{ username: 'test', is_admin: false, roles: [] }}
        makeAdmin={makeAdmin}
      />,
    );

    await userEvent.click(root.getByLabelText('more actions'));
    await userEvent.click(root.getByRole('menuitem', { name: 'Admin' }));
    await waitFor(() => true); // let async effects run
    expect(makeAdmin).toHaveBeenCalledWith(true);
  });
});
