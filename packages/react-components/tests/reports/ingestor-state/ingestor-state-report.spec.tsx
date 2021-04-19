import { render, waitFor, screen } from '@testing-library/react';
import React from 'react';
import { IngestorStateReport } from '../../../lib';
import { getIngestorLogs } from '../utils';

const getLogsPromise = async () => await getIngestorLogs();

it('smoke test', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<IngestorStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<IngestorStateReport getLogs={async () => await []} />);
  });

  expect(screen.queryByText('Ingestor State')).toBeFalsy();
});
