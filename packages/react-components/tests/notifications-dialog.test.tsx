import React from 'react';
import { NotificationsDialog } from '../lib';
import { render, fireEvent, screen } from '@testing-library/react';
import { Notification } from '../lib';

const notifications: Notification[] = [
  { id: 1, time: 'time', error: 'message', severity: 'High' },
  { id: 2, time: 'time', error: 'message', severity: 'Medium' },
  { id: 3, time: 'time', error: 'message', severity: 'Low' },
];

test('should call onClose when close button is clicked', () => {
  const mockOnClose = jest.fn();
  render(
    <NotificationsDialog
      onClose={mockOnClose}
      showNotificationsDialog={true}
      notifications={notifications}
    />,
  );
  fireEvent.click(screen.getByText(/CLOSE/i));
  expect(mockOnClose).toHaveBeenCalled();
});

test('should call onClose when close icon at the top is clicked', () => {
  const mockOnClose = jest.fn();
  render(
    <NotificationsDialog
      onClose={mockOnClose}
      showNotificationsDialog={true}
      notifications={notifications}
    />,
  );
  fireEvent.click(screen.getByLabelText(/close/i));
  expect(mockOnClose).toHaveBeenCalled();
});

test('should not render other severity levels when a particular level is selected', () => {
  const mockOnClose = jest.fn();
  render(
    <NotificationsDialog
      onClose={mockOnClose}
      showNotificationsDialog={true}
      notifications={notifications}
    />,
  );
  fireEvent.change(screen.getByLabelText(/filter-input/i).childNodes[1], {
    target: { value: 'High' },
  });
  expect(screen.queryByText('Medium')).toBeNull();
  expect(screen.queryByText('Low')).toBeNull();
});
