import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { EventEditDeletePopup } from './task-scheduled-edit-delete-popup';

describe('EventEditDeletePopup', () => {
  const currentValue = 'current';
  const allValue = 'all';
  const value = 'current';
  const deleting = true;
  const onChange = jasmine.createSpy('onChange');

  it('handles onChange event', () => {
    const { getByLabelText } = render(
      <EventEditDeletePopup
        currentValue={currentValue}
        allValue={allValue}
        value={value}
        deleting={deleting}
        onChange={onChange}
      />,
    );

    // Simulate a change event by clicking on a radio button
    fireEvent.click(getByLabelText(`All events`));

    // Check if the onChange function was called with the correct value
    expect(onChange).toHaveBeenCalled();
  });
});
