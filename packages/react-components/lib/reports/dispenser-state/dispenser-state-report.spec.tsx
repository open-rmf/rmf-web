import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import React from 'react';
import { getDispenserLogs, reportConfigProps } from '../utils.spec';
import { DispenserStateReport } from './dispenser-state-report';
import { TestLocalizationProvider } from '../../test/locale';

const getLogsPromise = async () => getDispenserLogs();

it('smoke test', async () => {
  await waitFor(() => {
    render(<DispenserStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  await waitFor(() => {
    render(<DispenserStateReport getLogs={async () => []} />);
  });

  expect(screen.queryByText('Dispenser State')).toBeFalsy();
});

it('calls the retrieve log function when the button is clicked', () => {
  const getLogsPromiseMock = jasmine.createSpy();
  const getLogsPromise = async () => {
    getLogsPromiseMock();
    return getDispenserLogs();
  };
  render(<DispenserStateReport getLogs={getLogsPromise} {...reportConfigProps} />, {
    wrapper: TestLocalizationProvider,
  });
  expect(screen.getByRole('button', { name: /Retrieve Logs/i })).toBeTruthy();
  fireEvent.click(screen.getByRole('button', { name: /Retrieve Logs/i }));
  expect(getLogsPromiseMock).toHaveBeenCalled();
});
