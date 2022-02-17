import { render } from '@testing-library/react';
import React from 'react';
import { TestLocalizationProvider } from '../../test/locale';
import { SearchLogForm } from './search-log-form';

describe('Search log form tests', () => {
  const logLabel = [
    { label: 'Web Server', value: 'web-server' },
    { label: 'RMF core', value: 'rmf-core' },
  ];

  it('smoke test', () => {
    render(<SearchLogForm logLabelValues={logLabel}></SearchLogForm>, {
      wrapper: TestLocalizationProvider,
    });
  });
});
