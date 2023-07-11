import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { TestLocalizationProvider } from '../../test/locale';
import { getDoorLogs, reportConfigProps } from '../utils.spec';
import { DoorStateReport } from './door-state-report';

const getLogsPromise = async () => getDoorLogs();

it('smoke test', async () => {
  await waitFor(() => {
    render(<DoorStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  await waitFor(() => {
    render(<DoorStateReport getLogs={async () => []} />);
  });

  expect(screen.queryByText('Door State')).toBeFalsy();
});

it('calls the retrieve log function when the button is clicked', () => {
  const getLogsPromiseMock = jasmine.createSpy();
  const getLogsPromise = async () => {
    getLogsPromiseMock();
    return getDoorLogs();
  };
  render(<DoorStateReport getLogs={getLogsPromise} {...reportConfigProps} />, {
    wrapper: TestLocalizationProvider,
  });
  expect(screen.getByRole('button', { name: /Retrieve Logs/i })).toBeTruthy();
  fireEvent.click(screen.getByRole('button', { name: /Retrieve Logs/i }));
  expect(getLogsPromiseMock).toHaveBeenCalled();
});
