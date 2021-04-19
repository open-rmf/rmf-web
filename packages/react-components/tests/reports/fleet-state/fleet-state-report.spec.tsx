import { render, waitFor, screen } from '@testing-library/react';
import React from 'react';
import { FleetStateReport } from '../../../lib';
import { getFleetLogs } from '../utils';

const getLogsPromise = async () => await getFleetLogs();

it('smoke test', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<FleetStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<FleetStateReport getLogs={async () => await []} />);
  });

  expect(screen.queryByText('Fleet State')).toBeFalsy();
});
