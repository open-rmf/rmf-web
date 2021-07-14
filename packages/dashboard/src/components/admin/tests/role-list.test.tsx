import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { RoleListCard } from '../role-list';

describe('Role List', () => {
  it('triggers callbackk when add/remove is clicked', () => {
    const cb = jest.fn();
    const root = render(<RoleListCard roles={[]} onAddRemoveClick={cb} />);
    userEvent.click(root.getByLabelText('Add/Remove'));
    expect(cb).toHaveBeenCalled();
  });

  it('renders list of roles', () => {
    const root = render(<RoleListCard roles={['role1', 'role2']} />);
    root.getByText('role1');
    root.getByText('role2');
  });
});
