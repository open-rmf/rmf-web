import { render } from '@testing-library/react';
import React from 'react';
import moment from 'moment';
import { DefaultDatesForm } from '../../lib';

describe('Default date form', () => {
  it('smoke test', () => {
    render(<DefaultDatesForm />);
  });

  it('places correctly initial values', () => {
    const root = render(<DefaultDatesForm />);
    const currentDate = moment(new Date()).format('MM/DD/yyyy HH:mm');
    const fromLogDate = root.container.querySelector('#fromLogDate-datetime-local');
    expect(fromLogDate?.getAttribute('value')).toBe(currentDate);
    const toLogDate = root.container.querySelector('#toLogDate-datetime-local');
    expect(toLogDate?.getAttribute('value')).toBe(currentDate);
  });
});
