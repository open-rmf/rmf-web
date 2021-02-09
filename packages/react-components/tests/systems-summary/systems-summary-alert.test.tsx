import React from 'react';
import { SystemSummaryAlert } from '../../lib';
import { render, fireEvent, screen } from '@testing-library/react';
import { notifications } from './test.utils';

test('button should be disabled when there are no notifications', () => {
  const root = render(<SystemSummaryAlert notifications={[]} />);
  fireEvent.click(screen.getByRole('button'));

  expect(root.container.querySelector('button')?.disabled).toEqual(true);
  // check that notification dialog box is not rendered via Filter text
  expect(screen.queryByText('Filter by severity')).toBeNull();
});

test('button should not be disabled when there are notifications', () => {
  const root = render(<SystemSummaryAlert notifications={notifications} />);
  expect(root.container.querySelector('button')?.disabled).toEqual(false);
});
