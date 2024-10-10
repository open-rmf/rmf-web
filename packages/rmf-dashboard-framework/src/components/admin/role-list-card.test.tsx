import { render as render_, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { describe, expect, it } from 'vitest';

import { AppControllerProvider } from '../../hooks/use-app-controller';
import { makeMockAppController } from '../../utils/test-utils.test';
import { RoleListCard } from './role-list-card';

const render = (ui: React.ReactNode) =>
  render_(<AppControllerProvider value={makeMockAppController()}>{ui}</AppControllerProvider>);

describe('Role List', () => {
  it('renders list of roles', async () => {
    const root = render(<RoleListCard getRoles={() => ['role1', 'role2']} />);
    await expect(waitFor(() => root.getByText('role1'))).resolves.not.toThrow();
    await expect(waitFor(() => root.getByText('role2'))).resolves.not.toThrow();
  });

  it('opens create role dialog when button is clicked', async () => {
    const root = render(<RoleListCard />);
    await userEvent.click(root.getByLabelText('create role'));
    expect(() => root.getByText('Create Role')).not.toThrow();
  });
});
