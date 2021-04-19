import { render, waitFor, screen } from '@testing-library/react';
import React from 'react';
import { HealthReport } from '../../../lib';
import { getHealthLogs } from '../utils';

const getLogsPromise = async () => await getHealthLogs();

it('smoke test', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<HealthReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<HealthReport getLogs={async () => await []} />);
  });

  expect(screen.queryByText('Health')).toBeFalsy();
});
