import React from 'react';
import { render } from '@testing-library/react';
import DateAndTimePickers from '../lib/date-time-picker';
import moment from 'moment';

describe('date time picker', () => {
  it('shows the correct Label', async () => {
    const handleDateChange = jest.fn();
    const root = render(
      <DateAndTimePickers
        name={'picker'}
        label={'Test'}
        value={handleDateChange}
        onChange={handleDateChange}
      />,
    );
    expect(await root.findByLabelText('Test')).toBeTruthy();
  });

  it('shows the current date if `date` parameter is undefined', () => {
    const handleDateChange = jest.fn();
    const currentDate = moment(new Date().toISOString().substr(0, 16)).format('yyyy/MM/DD HH:mm');

    const root = render(
      <DateAndTimePickers
        name={'picker'}
        label={'test'}
        value={undefined}
        onChange={handleDateChange}
      />,
    );
    const datePicker = root.container.querySelector('#picker-datetime-local');
    expect(datePicker?.getAttribute('value')).toBe(currentDate);
  });
});
