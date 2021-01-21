import { render } from '@testing-library/react';
import React from 'react';
import { SearchLogForm } from '../../lib';

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
    expect(root.getByText('ERROR')).toBeTruthy();
    expect(root.getByText('100')).toBeTruthy();
    const fromLogDate = root.container.querySelector('#fromLogDate-datetime-local');
    expect(fromLogDate?.getAttribute('value')).toBe(new Date().toISOString().substr(0, 16));
    const toLogDate = root.container.querySelector('#toLogDate-datetime-local');
    expect(toLogDate?.getAttribute('value')).toBe(new Date().toISOString().substr(0, 16));
  });
});
