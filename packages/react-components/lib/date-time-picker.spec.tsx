import { render } from '@testing-library/react';
import { format } from 'date-fns';
import React from 'react';
import { TextField } from '@material-ui/core';
import DateAndTimePickers from './date-time-picker';

describe('date time picker', () => {
  it('shows the correct Label', async () => {
    const handleDateChange = jasmine.createSpy();
    const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString().substr(0, 16);
    const root = render(
      <DateAndTimePickers
        label={'Test'}
        value={timestamp}
        onChange={handleDateChange}
        renderInput={(props) => <TextField {...props} />}
      />,
    );
    expect(await root.findByText('Test')).toBeTruthy();
  });

  it('shows the current date if `date` parameter is undefined', () => {
    const handleDateChange = jasmine.createSpy();
    const currentDate = format(new Date(), 'MM/dd/yyyy HH:mm');

    const root = render(
      <DateAndTimePickers
        label={'test'}
        value={undefined}
        onChange={handleDateChange}
        renderInput={(props) => <TextField {...props} />}
      />,
    );
    const datePicker = root.container.querySelector('.MuiInputBase-input.MuiInput-input');
    expect(datePicker?.getAttribute('value')).toBe(currentDate);
  });
});
