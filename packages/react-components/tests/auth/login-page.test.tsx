import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { LoginPage } from '../../lib';

test('triggers onLoginClick when button is clicked', () => {
  const handler = jest.fn();
  const root = render(<LoginPage logo="" onLoginClick={handler} />);
  userEvent.click(root.getByText('Login with RMF'));
  expect(handler).toBeCalled();
});
