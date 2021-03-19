import { render, screen } from '@testing-library/react';
import React from 'react';
import MainMenu from '../main-menu';
import userEvent from '@testing-library/user-event';

it('renders without crashing', () => {
  const root = render(<MainMenu pushView={jest.fn()} setFilter={jest.fn()} />);
  root.unmount();
});

it('should call pushView and setFilter when Doors button is clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Doors'));

  expect(mockPushView).toBeCalled();
  expect(mockSetFilter).toBeCalled();
});

it('should call pushView and setFilter when Lifts button is clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Lifts'));

  expect(mockPushView).toBeCalled();
  expect(mockSetFilter).toBeCalled();
});

it('should call pushView and setFilter when Robots button is clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Robots'));

  expect(mockPushView).toBeCalled();
  expect(mockSetFilter).toBeCalled();
});

it('should call pushView and setFilter when Dispensers button is clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Dispensers'));

  expect(mockPushView).toBeCalled();
  expect(mockSetFilter).toBeCalled();
});

it('should call pushView and setFilter when Negotiations button is clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Negotiations'));

  expect(mockPushView).toBeCalled();
  expect(mockSetFilter).toBeCalled();
});

it('should call pushView and setFilter when Plans button is clicked', () => {
  const mockPushView = jest.fn();
  const mockSetFilter = jest.fn();
  render(<MainMenu pushView={mockPushView} setFilter={mockSetFilter} />);

  userEvent.click(screen.getByText('Plans'));

  expect(mockPushView).toBeCalled();
  expect(mockSetFilter).toBeCalled();
});
