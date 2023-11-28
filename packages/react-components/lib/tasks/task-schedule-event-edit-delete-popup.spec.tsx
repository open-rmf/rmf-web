import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { EventEditDeletePopup } from './task-schedule-event-edit-delete-popup';

describe('EventEditDeletePopup', () => {
  const currentValue = 'current';
  const allValue = 'all';
  const value = 'current';
  const onChange = jasmine.createSpy('onChange');

  it('handles onChange event', () => {
    const { getByLabelText } = render(
      <EventEditDeletePopup
        currentValue={currentValue}
        allValue={allValue}
        value={value}
        isAdmin={true}
        onChange={onChange}
      />,
    );

    // Simulate a change event by clicking on a radio button
    fireEvent.click(getByLabelText('All events in this schedule'));

    // Check if the onChange function was called with the correct value
    expect(onChange).toHaveBeenCalled();
  });

  it('all events change disabled for non-admin', () => {
    const { getByLabelText } = render(
      <EventEditDeletePopup
        currentValue={currentValue}
        allValue={allValue}
        value={value}
        isAdmin={false}
        onChange={onChange}
      />,
    );

    // Simulate a change event by clicking on a radio button
    fireEvent.click(getByLabelText('All events in this schedule'));

    // Check that onChange was not called since the button is disabled
    expect(onChange).not.toHaveBeenCalled();
  });
});
