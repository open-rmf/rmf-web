import { render, waitFor, screen } from '@testing-library/react';
import React from 'react';
import { LiftStateReport } from '../../../lib';
import { getLiftLogs } from '../utils';

const getLogsPromise = async () => await getLiftLogs();

it('smoke test', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<LiftStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<LiftStateReport getLogs={async () => await []} />);
  });

  expect(screen.queryByText('Lift State')).toBeFalsy();
});
