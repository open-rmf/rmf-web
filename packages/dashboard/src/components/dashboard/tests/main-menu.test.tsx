import { render, screen } from '@testing-library/react';
import React from 'react';
import MainMenu from '../main-menu';
import userEvent from '@testing-library/user-event';

it('renders without crashing', () => {
  const root = render(<MainMenu pushView={jest.fn()} setFilter={jest.fn()} />);
  root.unmount();
});

it('should call pushView and setFilter when buttons are clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Doors'));
  userEvent.click(screen.getByText('Lifts'));
  userEvent.click(screen.getByText('Robots'));
  userEvent.click(screen.getByText('Dispensers'));
  userEvent.click(screen.getByText('Negotiations'));
  userEvent.click(screen.getByText('Plans'));

  expect(mockPushView).toBeCalledTimes(6);
  expect(mockSetFilter).toBeCalledTimes(6);
});
