import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { AlertDialog } from '../lib';

test('triggers onPositiveClick', () => {
  const handler = jest.fn();
  const root = render(
    <AlertDialog open={true} title="test" variant="warn" onPositiveClick={handler} />,
  );
  userEvent.click(root.getByText('OK'));
  expect(handler).toHaveBeenCalledTimes(1);
});

test('negative button is shown only when negativeText is provided', () => {
  const root = render(<AlertDialog open={true} title="test" variant="warn" />);
  expect(root.queryAllByRole('button')).toHaveLength(1);
  root.unmount();

  const root2 = render(
    <AlertDialog open={true} title="test" variant="warn" negativeText="Cancel" />,
  );
  expect(root2.queryAllByRole('button')).toHaveLength(2);
});

test('triggers onNegativeClick', () => {
  const handler = jest.fn();
  const root = render(
    <AlertDialog
      open={true}
      title="test"
      variant="warn"
      negativeText="Cancel"
      onNegativeClick={handler}
    />,
  );
  userEvent.click(root.getByText('Cancel'));
  expect(handler).toHaveBeenCalledTimes(1);
});

test('has close button when onCloseClick is provided', () => {
  const root = render(
    <AlertDialog open={true} title="test" variant="warn" onCloseClick={jest.fn()} />,
  );
  expect(root.getByLabelText('close')).toBeTruthy();
});
