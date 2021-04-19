import { screen, render, RenderResult, cleanup } from '@testing-library/react';
import React from 'react';
import moment from 'moment';
import { DoorStateReportTable } from '../../../lib';
import { getDoorLogs } from '../utils';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString();

describe('Door table test', () => {
  let root: RenderResult;
  beforeEach(() => {
    root = render(<DoorStateReportTable rows={getDoorLogs()} />);
  });

  afterEach(cleanup);

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('door-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(moment(timestamp).format('lll'));
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('tr').length;
    // -3. from the tr of the table header, filter and pagination table
    expect(allRows - 3).toBe(100);
  });

  it('shows titles correctly', () => {
    expect(screen.queryByText('Name')).toBeTruthy();
    expect(screen.queryByText('State')).toBeTruthy();
    expect(screen.queryByText('Timestamp')).toBeTruthy();
  });
});
