import React from 'react';
import { SystemSummaryAlert } from '../../lib';
import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { notifications } from './test.utils';

it('button should be disabled when there are no notifications', () => {
  const mockOnNotificationDismiss = jasmine.createSpy();
  const root = render(
    <SystemSummaryAlert notifications={[]} onNotificationsDismiss={mockOnNotificationDismiss} />,
  );
  userEvent.click(screen.getByRole('button'));

  expect(root.container.querySelector('button')?.disabled).toEqual(true);
  // check that notification dialog box is not rendered via Filter text
  expect(screen.queryByText('Filter by severity')).toBeNull();
});

it('button should not be disabled when there are notifications', () => {
  const mockOnNotificationDismiss = jasmine.createSpy();
  const root = render(
    <SystemSummaryAlert
      notifications={notifications}
      onNotificationsDismiss={mockOnNotificationDismiss}
    />,
  );
  expect(root.container.querySelector('button')?.disabled).toEqual(false);
});
