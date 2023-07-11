import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import { act } from 'react-dom/test-utils';
import React from 'react';
import { TestLocalizationProvider } from '../../test/locale';
import { getLiftLogs, reportConfigProps } from '../utils.spec';
import { LiftStateReport } from './lift-state-report';

describe('Lift State report', () => {
  const getLogsPromise = async () => getLiftLogs();

  it('smoke test', async () => {
    act(() => {
      render(<LiftStateReport getLogs={getLogsPromise} />);
    });
    await waitFor(() => {
      render(<LiftStateReport getLogs={getLogsPromise} />);
    });
  });

  it('doesn`t shows the table when logs list is empty', async () => {
    act(() => {
      render(<LiftStateReport getLogs={async () => []} />);
    });
    await waitFor(() => {
      render(<LiftStateReport getLogs={async () => []} />);
    });

    expect(screen.queryByText('Lift State')).toBeFalsy();
  });

  it('calls the retrieve log function when the button is clicked', async () => {
    const getLogsPromiseMock = jasmine.createSpy();
    const getLogsPromise = async () => {
      getLogsPromiseMock();
      return getLiftLogs();
    };
    await act(async () => {
      render(<LiftStateReport getLogs={getLogsPromise} {...reportConfigProps} />, {
        wrapper: TestLocalizationProvider,
      });
    });
    expect(screen.getByRole('button', { name: /Retrieve Logs/i })).toBeTruthy();
    fireEvent.click(screen.getByRole('button', { name: /Retrieve Logs/i }));
    expect(getLogsPromiseMock).toHaveBeenCalled();
  });
});
