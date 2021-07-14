import { render } from '@testing-library/react';
import React from 'react';
import { RoleListCard } from '../role-list';

describe('Role List', () => {
  it('renders list of roles', () => {
    const root = render(<RoleListCard roles={['role1', 'role2']} />);
    root.getByText('role1');
    root.getByText('role2');
  });
});
