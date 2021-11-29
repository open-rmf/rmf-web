import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import App from './App';

test('should render app', async () => {
  const node = render(<App />);
  expect(await waitFor(() => node.container.querySelector('#appbar'))).toBeTruthy();
  expect(screen.getByLabelText('task-display')).toBeTruthy();
  expect(screen.getByLabelText('task-form')).toBeTruthy();
});
