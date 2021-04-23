import { render, waitFor, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { IngestorStateReport } from '../../../lib';
import { getIngestorLogs } from '../utils';

const getLogsPromise = async () => await getIngestorLogs();

it('smoke test', async () => {
  await waitFor(() => {
    render(<IngestorStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  await waitFor(() => {
    render(<IngestorStateReport getLogs={async () => await []} />);
  });

  expect(screen.queryByText('Ingestor State')).toBeFalsy();
});

it('calls the retrieve log function when the button is clicked', async () => {
  const getLogsPromiseMock = jasmine.createSpy();
  const getLogsPromise = async () => {
    getLogsPromiseMock();
    return await getIngestorLogs();
  };
  render(<IngestorStateReport getLogs={getLogsPromise} />);
  expect(screen.getByRole('button', { name: /Retrieve Logs/i })).toBeTruthy();
  userEvent.click(screen.getByRole('button', { name: /Retrieve Logs/i }));
  expect(getLogsPromiseMock).toHaveBeenCalled();
});
