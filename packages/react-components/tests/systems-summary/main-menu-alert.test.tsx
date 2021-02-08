import React from 'react';
import { SystemSummaryAlert } from '../../lib';
import { render } from '@testing-library/react';
import { notifications } from './test.utils';

test('button should be disabled when there are no notifications', () => {
  const root = render(<SystemSummaryAlert notifications={[]} />);
  expect(root.container.querySelector('button')?.disabled).toEqual(true);
});

test('button should not be disabled when there are notifications', () => {
  const root = render(<SystemSummaryAlert notifications={notifications} />);
  expect(root.container.querySelector('button')?.disabled).toEqual(false);
});
