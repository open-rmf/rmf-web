import { cleanup, render, RenderResult, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { format } from 'date-fns';
import React from 'react';
import { getDispenserLogs } from '../utils.spec';
import { DispenserStateReportTable } from './dispenser-state-report-table';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001');

describe('Dispenser table test', () => {
  let root: RenderResult;
  let mockAddMoreRows: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    mockAddMoreRows = jasmine.createSpy();
    root = render(
      <DispenserStateReportTable rows={getDispenserLogs()} addMoreRows={mockAddMoreRows} />,
    );
  });

  afterEach(cleanup);

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('dispenser-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(format(timestamp, 'MMM dd yyyy hh:mm aaa'));
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(100);
  });

  it('shows titles correctly', () => {
    expect(screen.queryByText('Guid')).toBeTruthy();
    expect(screen.queryByText('State')).toBeTruthy();
    expect(screen.queryByText('Timestamp')).toBeTruthy();
  });

  it('executes the addMoreRows', () => {
    const nextPageButton = screen.queryByLabelText('Go to next page');
    nextPageButton && userEvent.click(nextPageButton);
    expect(mockAddMoreRows).toHaveBeenCalled();
  });
});
