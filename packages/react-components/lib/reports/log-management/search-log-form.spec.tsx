import { render } from '@testing-library/react';
import { format } from 'date-fns';
import React from 'react';
import { SearchLogForm } from './search-log-form';

describe('Search log form tests', () => {
  const logLabel = [
    { label: 'Web Server', value: 'web-server' },
    { label: 'RMF core', value: 'rmf-core' },
  ];

  it('smoke test', () => {
    render(<SearchLogForm logLabelValues={logLabel}></SearchLogForm>);
  });

  it('places correctly initial values', () => {
    const root = render(<SearchLogForm logLabelValues={logLabel}></SearchLogForm>);
    expect(root.getByText('ALL')).toBeTruthy();
    const currentDate = format(new Date(), 'MM/dd/yyyy HH:mm');
    const getInputs = root.container.querySelectorAll('.MuiInputBase-input.MuiInput-input');
    expect(getInputs[0].getAttribute('value')).toBe(currentDate);
    expect(getInputs[1].getAttribute('value')).toBe(currentDate);
  });
});
