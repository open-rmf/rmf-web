import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import React from 'react';
import { TestLocalizationProvider } from '../../test/locale';
import { getFleetLogs, reportConfigProps } from '../utils.spec';
import { FleetStateReport } from './fleet-state-report';

describe('Fleet State report', () => {
  const getLogsPromise = async () => getFleetLogs();

  it('smoke test', async () => {
    await waitFor(() => {
      render(<FleetStateReport getLogs={getLogsPromise} />);
    });
  });

  it('doesn`t shows the table when logs list is empty', async () => {
    await waitFor(() => {
      render(<FleetStateReport getLogs={async () => []} />);
    });

    expect(screen.queryByText('Fleet State')).toBeFalsy();
  });

  it('calls the retrieve log function when the button is clicked', async () => {
    const getLogsPromiseMock = jasmine.createSpy();
    const getLogsPromise = async () => {
      getLogsPromiseMock();
      return getFleetLogs();
    };
    render(<FleetStateReport getLogs={getLogsPromise} {...reportConfigProps} />, {
      wrapper: TestLocalizationProvider,
    });
    expect(screen.getByRole('button', { name: /Retrieve Logs/i })).toBeTruthy();
    fireEvent.click(screen.getByRole('button', { name: /Retrieve Logs/i }));
    expect(getLogsPromiseMock).toHaveBeenCalled();
  });
});
