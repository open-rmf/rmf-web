import { render, waitFor, screen } from '@testing-library/react';
import React from 'react';
import { DoorStateReport } from '../../../lib';
import { getDoorLogs } from '../utils';

const getLogsPromise = async () => await getDoorLogs();

it('smoke test', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<DoorStateReport getLogs={getLogsPromise} />);
  });
});

it('doesn`t shows the table when logs list is empty', async () => {
  // Added the waitFor because this component is updating a state inside a useEffect.
  await waitFor(() => {
    render(<DoorStateReport getLogs={async () => await []} />);
  });

  expect(screen.queryByText('Door State')).toBeFalsy();
});
