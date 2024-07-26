import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { EventEditDeletePopup } from './task-schedule-event-edit-delete-popup';

describe('EventEditDeletePopup', () => {
  const currentValue = 'current';
  const allValue = 'all';
  const value = 'current';
  const onChange = jest.fn();

  it('handles onChange event', () => {
    const { getByLabelText } = render(
      <EventEditDeletePopup
        currentValue={currentValue}
        allValue={allValue}
        value={value}
        onChange={onChange}
      />,
    );

    // Simulate a change event by clicking on a radio button
    fireEvent.click(getByLabelText('All events in this schedule'));

    // Check if the onChange function was called with the correct value
    expect(onChange).toHaveBeenCalled();
  });
});
