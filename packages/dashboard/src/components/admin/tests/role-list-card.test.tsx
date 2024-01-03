import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { RoleListCard } from '../role-list-card';

describe('Role List', () => {
  it('renders list of roles', async () => {
    const root = render(<RoleListCard getRoles={() => ['role1', 'role2']} />);
    await expect(waitFor(() => root.getByText('role1'))).resolves.not.toThrow();
    await expect(waitFor(() => root.getByText('role2'))).resolves.not.toThrow();
  });

  it('opens create role dialog when button is clicked', async () => {
    const root = render(<RoleListCard />);
    const user = userEvent.setup();
    await user.click(screen.getByTestId('create-role'));
    expect(() => screen.getByTestId('create-role-dialog')).not.toThrow();
  });
});
