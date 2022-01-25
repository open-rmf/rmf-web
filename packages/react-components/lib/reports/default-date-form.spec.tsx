import { render } from '@testing-library/react';
import { format } from 'date-fns';
import React from 'react';
import { DefaultDatesForm } from './default-dates-form';
import { reportConfigProps } from './utils.spec';

describe('Default date form', () => {
  it('smoke test', () => {
    render(<DefaultDatesForm />);
  });

  it('places correctly initial values', () => {
    const currentDate = format(new Date(), 'MM/dd/yyyy HH:mm');
    const newConfig = { ...reportConfigProps, fromLogDate: new Date(), toLogDate: new Date() };
    const root = render(<DefaultDatesForm {...newConfig} />);
    const getInputs = root.container.querySelectorAll('.MuiOutlinedInput-input.MuiInputBase-input');
    expect(getInputs[0].getAttribute('value')).toBe(currentDate);
    expect(getInputs[1].getAttribute('value')).toBe(currentDate);
  });
});
