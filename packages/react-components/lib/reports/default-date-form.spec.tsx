import { render } from '@testing-library/react';
import React from 'react';
import { TestLocalizationProvider } from '../test/locale';
import { DefaultDatesForm } from './default-dates-form';
import { reportConfigProps } from './utils.spec';

describe('Default date form', () => {
  it('places correctly initial values', () => {
    const date = new Date();
    const newConfig = { ...reportConfigProps, fromLogDate: date, toLogDate: date };
    const root = render(<DefaultDatesForm {...newConfig} />, { wrapper: TestLocalizationProvider });
    const fromInput = root.getByLabelText('From');
    expect(fromInput.getAttribute('data-unix')).toBe(date.valueOf().toString());
    const toInput = root.getByLabelText('To');
    expect(toInput.getAttribute('data-unix')).toBe(date.valueOf().toString());
  });
});
