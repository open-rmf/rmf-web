import React from 'react';
import { render } from '@testing-library/react';
import DateAndTimePickers from '../lib/date-time-picker';

describe('date time picker', () => {
  it('shows error message', async () => {
    const handleDateChange = jest.fn();
    const root = render(
      <DateAndTimePickers
        error={'Test Error'}
        date={'2021-01-21T17:42'}
        name={'picker'}
        label={'test'}
        handleDateChange={handleDateChange}
      />,
    );
    expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    expect(root.container.querySelector('#picker-datetime-local-helper-text')?.innerHTML).toBe(
      'Test Error',
    );
  });

  it('shows the correct Label', async () => {
    const handleDateChange = jest.fn();
    const root = render(
      <DateAndTimePickers name={'picker'} label={'Test'} handleDateChange={handleDateChange} />,
    );
    expect(await root.findByLabelText('Test')).toBeTruthy();
  });

  it('shows the current date if `date` parameter is undefined', () => {
    const handleDateChange = jest.fn();
    const root = render(
      <DateAndTimePickers name={'picker'} label={'test'} handleDateChange={handleDateChange} />,
    );
    const datePicker = root.container.querySelector('#picker-datetime-local');
    expect(datePicker?.getAttribute('value')).toBe(new Date().toISOString().substr(0, 16));
  });
});
