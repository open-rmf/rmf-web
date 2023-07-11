import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import React from 'react';
import { TestLocalizationProvider } from '../../test/locale';
import { getIngestorLogs, reportConfigProps } from '../utils.spec';
import { IngestorStateReport } from './ingestor-state-report';

describe('Ingestor State report', () => {
  const getLogsPromise = async () => getIngestorLogs();

  it('smoke test', async () => {
    await waitFor(() => {
      render(<IngestorStateReport getLogs={getLogsPromise} />);
    });
  });

  it('doesn`t shows the table when logs list is empty', async () => {
    await waitFor(() => {
      render(<IngestorStateReport getLogs={async () => []} />);
    });

    expect(screen.queryByText('Ingestor State')).toBeFalsy();
  });

  it('calls the retrieve log function when the button is clicked', async () => {
    const getLogsPromiseMock = jasmine.createSpy();
    const getLogsPromise = async () => {
      getLogsPromiseMock();
      return getIngestorLogs();
    };
    render(<IngestorStateReport getLogs={getLogsPromise} {...reportConfigProps} />, {
      wrapper: TestLocalizationProvider,
    });
    expect(screen.getByRole('button', { name: /Retrieve Logs/i })).toBeTruthy();
    fireEvent.click(screen.getByRole('button', { name: /Retrieve Logs/i }));
    expect(getLogsPromiseMock).toHaveBeenCalled();
  });
});
