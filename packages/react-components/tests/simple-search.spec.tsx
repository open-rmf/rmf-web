import React from 'react';
import { render, screen } from '@testing-library/react';
import { SimpleSearch } from '../lib/index';
import userEvent, { TargetElement } from '@testing-library/user-event';

it('should call onChange when input text is updated', () => {
  const mockOnChange = jasmine.createSpy();
  const mockOnClick = jasmine.createSpy();
  render(
    <SimpleSearch disabled={false} value={''} onChange={mockOnChange} onClick={mockOnClick} />,
  );

  userEvent.paste(
    screen.getByLabelText('text-input').childNodes[1].childNodes[0] as TargetElement,
    'new text',
  );

  expect(mockOnChange).toHaveBeenCalled();
});

it('should not call onChange when input text is disabled', () => {
  const mockOnChange = jasmine.createSpy();
  const mockOnClick = jasmine.createSpy();
  render(<SimpleSearch disabled={true} value={''} onChange={mockOnChange} onClick={mockOnClick} />);

  userEvent.paste(
    screen.getByLabelText('text-input').childNodes[1].childNodes[0] as TargetElement,
    'new text',
  );

  expect(mockOnChange).not.toHaveBeenCalled();
});

it('should call onClick when delete search term button is clicked', () => {
  const mockOnChange = jasmine.createSpy();
  const mockOnClick = jasmine.createSpy();
  render(
    <SimpleSearch
      disabled={false}
      value={'search_value'}
      onChange={mockOnChange}
      onClick={mockOnClick}
    />,
  );

  userEvent.click(screen.getByRole('button'));

  expect(mockOnClick).toHaveBeenCalled();
});

it('should not call onClick when delete search term button is disabled', () => {
  const mockOnChange = jasmine.createSpy();
  const mockOnClick = jasmine.createSpy();
  render(<SimpleSearch disabled={true} value={''} onChange={mockOnChange} onClick={mockOnClick} />);

  userEvent.click(screen.getByRole('button'));

  expect(mockOnClick).not.toHaveBeenCalled();
});
