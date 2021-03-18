import React from 'react';
import { render, screen } from '@testing-library/react';
import { SimpleFilter } from '../lib/index';
import userEvent, { TargetElement } from '@testing-library/user-event';

it('should call onChange when input text is updated', () => {
  const mockOnChange = jasmine.createSpy();
  render(<SimpleFilter value={''} onChange={mockOnChange} />);

  userEvent.paste(
    screen.getByLabelText('text-input').childNodes[1].childNodes[0] as TargetElement,
    'new text',
  );

  expect(mockOnChange).toHaveBeenCalled();
});
