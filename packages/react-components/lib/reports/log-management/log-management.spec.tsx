import { render, waitFor } from '@testing-library/react';
import React from 'react';
import { LogManagement } from './log-management';
import { LogRowsType } from './log-table';
import { TestLocalizationProvider } from '../../test/locale';

// TODO(AA): To remove after
// https://github.com/testing-library/react-testing-library/issues/1216
// has been resolved.
// Workaround for "Warning: An update to ComponentName inside a test was not
// wrapped in act(...)."
const originalError = console.error;
beforeAll(() => {
  console.error = (...args) => {
    if (/Warning.*not wrapped in act/.test(args[0])) {
      return;
    }
    originalError.call(console, ...args);
  };
});

afterAll(() => {
  console.error = originalError;
});

const getLogLabels = () => [
  { label: 'Web Server', value: 'web-server' },
  { label: 'RMF core', value: 'rmf-core' },
];

const getLogs = () => {
  const rows: LogRowsType = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      message: 'Test' + i,
      level: 'WARN',
      created: new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString(),
      container: { id: i, name: 'container' },
    });
  }
  return rows;
};

const getLogsPromise = async () => await getLogs();
const getLabelsPromise = async () => await getLogLabels();

it('smoke test', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<LogManagement getLabels={getLabelsPromise} getLogs={getLogsPromise} />, {
      wrapper: TestLocalizationProvider,
    });
  });
});
