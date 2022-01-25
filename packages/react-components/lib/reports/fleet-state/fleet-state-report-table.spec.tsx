import { cleanup, render, RenderResult, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { format } from 'date-fns';
import React from 'react';
import { getFleetLogs } from '../utils.spec';
import { FleetStateReportTable } from './fleet-state-report-table';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001');

describe('Fleet table test', () => {
  let root: RenderResult;
  let mockAddMoreRows: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    mockAddMoreRows = jasmine.createSpy();
    root = render(<FleetStateReportTable rows={getFleetLogs()} addMoreRows={mockAddMoreRows} />);
  });

  afterEach(cleanup);

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('fleet-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(format(timestamp, 'MMM dd yyyy hh:mm aaa'));
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(100);
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

  it('executes the addMoreRows', () => {
    const nextPageButton = screen.queryByTitle('Go to next page');
    nextPageButton && userEvent.click(nextPageButton);
    expect(mockAddMoreRows).toHaveBeenCalled();
  });
});
