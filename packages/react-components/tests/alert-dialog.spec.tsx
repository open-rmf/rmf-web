import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { AlertDialog } from '../lib';

it('triggers onPositiveClick', () => {
  const handler = {
    onClick: () => {},
  };

  spyOn(handler, 'onClick');
  const root = render(
    <AlertDialog open={true} title="test" variant="warn" onPositiveClick={handler.onClick} />,
  );
  userEvent.click(root.getByText('OK'));
  expect(handler.onClick).toHaveBeenCalledTimes(1);
});

it('negative button is shown only when negativeText is provided', () => {
  const root = render(<AlertDialog open={true} title="test" variant="warn" />);
  expect(root.queryAllByRole('button')).toHaveSize(1);
  root.unmount();

  const root2 = render(
    <AlertDialog open={true} title="test" variant="warn" negativeText="Cancel" />,
  );
  expect(root2.queryAllByRole('button')).toHaveSize(2);
});

it('triggers onNegativeClick', () => {
  const handler = {
    onClick: () => {},
  };

  spyOn(handler, 'onClick');
  const root = render(
    <AlertDialog
      open={true}
      title="test"
      variant="warn"
      negativeText="Cancel"
      onNegativeClick={handler.onClick}
    />,
  );
  userEvent.click(root.getByText('Cancel'));
  expect(handler.onClick).toHaveBeenCalledTimes(1);
});

it('has close button when onCloseClick is provided', () => {
  const root = render(
    <AlertDialog open={true} title="test" variant="warn" onCloseClick={(e) => {}} />,
  );
  expect(root.getByLabelText('close')).toBeTruthy();
});
