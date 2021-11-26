import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { TaskForm } from '../task-form';

const mockPlaceNames = ['place1', 'place2', 'place3'];

describe('task form', () => {
  it('should show success snack bar message when destination has been selected on submit', () => {
    render(<TaskForm placeNames={mockPlaceNames} onFetchTask={jest.fn()} />);
    userEvent.click(screen.getByText('place2'));
    userEvent.click(screen.getByText('Submit'));
    expect(waitFor(() => screen.getByText('Task Submitted Successfully'))).toBeTruthy();
  });

  it('should show error snack bar message if destination is not selected on submit', () => {
    render(<TaskForm placeNames={mockPlaceNames} onFetchTask={jest.fn()} />);
    userEvent.click(screen.getByText('Submit'));
    expect(waitFor(() => screen.getByText('You must select a destination'))).toBeTruthy();
  });
});
