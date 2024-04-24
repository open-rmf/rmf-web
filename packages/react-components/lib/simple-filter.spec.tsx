import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { SimpleFilter } from './simple-filter';

it('should call onChange when input text is updated', async () => {
  const mockOnChange = jest.fn();
  render(<SimpleFilter value={''} onChange={mockOnChange} />);

  await userEvent.click(screen.getByLabelText('text-input').childNodes[1].childNodes[0] as Element);
  await userEvent.paste('new text');

  expect(mockOnChange).toHaveBeenCalled();
});
