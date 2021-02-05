import React from 'react';
import { MainMenuAlert } from '../../lib';
import { render } from '@testing-library/react';
import { notifications, deletedNotifications } from './test.utils';

test('button should be disabled when there are no notifications', () => {
  const root = render(
    <MainMenuAlert notifications={notifications} deletedNotifications={deletedNotifications} />,
  );
  expect(root.container.querySelector('button')?.disabled).toEqual(true);
});

test('button should not be disabled when there are notifications', () => {
  const root = render(<MainMenuAlert notifications={notifications} deletedNotifications={[]} />);
  expect(root.container.querySelector('button')?.disabled).toEqual(false);
});
