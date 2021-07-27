import React from 'react';
import { render, cleanup, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { RadioButtonGroup } from './radio-button-group';

describe('Radio Group', () => {
  it('renders and is clickable', () => {
    const mockOnClick = jasmine.createSpy();
    const options = ['option1', 'option2', 'option3'];
    render(
      <RadioButtonGroup
        formLabel="Options"
        options={options}
        radioGroupName="test"
        onHandleChange={mockOnClick}
      />,
    );
    const option2 = screen.getByText(/option2/i);
    userEvent.click(option2);
    expect(mockOnClick).toHaveBeenCalled();
    cleanup();
  });
});
