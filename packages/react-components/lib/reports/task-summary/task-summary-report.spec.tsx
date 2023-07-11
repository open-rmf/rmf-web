import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import React from 'react';
import { TestLocalizationProvider } from '../../test/locale';
import { getTaskSummaryLogs, reportConfigProps } from '../utils.spec';
import { TaskSummaryReport } from './task-summary-report';

const getLogsPromise = async () => getTaskSummaryLogs();

it('smoke test', async () => {
  await waitFor(() => {
    render(<TaskSummaryReport getLogs={getLogsPromise} />);
  });
});

it('does not show the table when the logs list is empty', async () => {
  await waitFor(() => {
    render(<TaskSummaryReport getLogs={async () => []} />);
  });

  expect(screen.queryByText('Task Summary')).toBeFalsy();
});

it('calls the Retrieve Logs function when the button is clicked', async () => {
  const getLogsPromiseMock = jasmine.createSpy();
  const getLogsPromise = async () => {
    getLogsPromiseMock();
    return getTaskSummaryLogs();
  };

  render(<TaskSummaryReport getLogs={getLogsPromise} {...reportConfigProps} />, {
    wrapper: TestLocalizationProvider,
  });
  const retrieveLogsButton = screen.getByRole('button', { name: /Retrieve Logs/i });
  expect(retrieveLogsButton).toBeTruthy();
  fireEvent.click(retrieveLogsButton);
  expect(getLogsPromiseMock).toHaveBeenCalled();
});
