import { screen, render, RenderResult, cleanup } from '@testing-library/react';
import React from 'react';
import moment from 'moment';
import { FleetStateReportTable } from '../../../lib';
import { getFleetLogs } from '../utils';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString();

describe('Fleet table test', () => {
  let root: RenderResult;
  beforeEach(() => {
    root = render(<FleetStateReportTable rows={getFleetLogs()} />);
  });

  afterEach(cleanup);

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('fleet-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(moment(timestamp).format('lll'));
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('tr').length;
    // -3. from the tr of the table header, filter and pagination table
    expect(allRows - 3).toBe(100);
  });

  it('shows titles correctly', () => {
    expect(screen.queryByText('Fleet')).toBeTruthy();
    expect(screen.queryByText('Robot')).toBeTruthy();
    expect(screen.queryByText('Battery')).toBeTruthy();
    expect(screen.queryByText('Mode')).toBeTruthy();
    expect(screen.queryByText('Model')).toBeTruthy();
    expect(screen.queryByText('TaskID')).toBeTruthy();
    expect(screen.queryByText('Timestamp')).toBeTruthy();
  });
});
