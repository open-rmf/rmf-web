import { render as render_, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { MemoryRouter } from 'react-router';
import { UserListCard } from '../user-list-card';

const render = (component: React.ReactNode) => render_(<MemoryRouter>{component}</MemoryRouter>);

describe('UserListCard', () => {
  it('opens delete dialog when button is clicked', async () => {
    const root = render(<UserListCard searchUsers={() => ['user1']} />);
    await waitFor(() => root.getByText('Delete'));
    await userEvent.click(root.getByText('Delete'));
    expect(await root.findByText('Confirm Delete')).toBeInTheDocument();
  });

  it('opens create user dialog when button is clicked', async () => {
    const root = render(<UserListCard />);
    await userEvent.click(root.getByLabelText('create user'));
    expect(await root.findByText('Create User')).toBeInTheDocument();
  });

  it('calls searchUser when search box changes', async () => {
    const searchUsers = jest.fn(() => []);
    const root = render(<UserListCard searchUsers={searchUsers} />);
    await userEvent.type(root.getByLabelText('Search Users'), 'test');
    await waitFor(() => expect(searchUsers).toHaveBeenCalledTimes(2), { timeout: 1000 });
    expect((searchUsers.mock.calls[1] as any[])[0]).toBe('test');
  });
});
