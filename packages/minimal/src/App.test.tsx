import React from 'react';
import { render, screen } from '@testing-library/react';
import App from './App';

test('should render app', () => {
  const node = render(<App />);
  expect(node.container.querySelector('#appbar')).toBeTruthy();
  expect(screen.getByLabelText('task-display')).toBeTruthy();
  expect(screen.getByLabelText('task-form')).toBeTruthy();
});
