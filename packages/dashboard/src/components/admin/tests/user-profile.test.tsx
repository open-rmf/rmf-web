import { render, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { UserProfileCard } from '../user-profile';

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
    const makeAdmin = jest.fn();
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
