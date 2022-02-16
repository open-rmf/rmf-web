import { render } from '@testing-library/react';
import React from 'react';
import { LocalizationProvider } from '..';
import { DefaultDatesForm } from './default-dates-form';
import { reportConfigProps } from './utils.spec';

describe('Default date form', () => {
  it('places correctly initial values', async () => {
    const date = new Date();
    const newConfig = { ...reportConfigProps, fromLogDate: date, toLogDate: date };
    const root = render(<DefaultDatesForm {...newConfig} />, { wrapper: LocalizationProvider });
    const fromInput = await root.findByLabelText('From');
    expect(fromInput.getAttribute('data-unix')).toBe(date.valueOf().toString());
    const toInput = await root.findByLabelText('To');
    expect(toInput.getAttribute('data-unix')).toBe(date.valueOf().toString());
  });
});
