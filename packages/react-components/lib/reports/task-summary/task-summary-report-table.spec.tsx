import { cleanup, render, RenderResult, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { format } from 'date-fns';
import React from 'react';
import { getTaskSummaryLogs } from '../utils.spec';
import { TaskSummaryReportTable } from './task-summary-report-table';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001');

describe('Task Summary Table Test', () => {
  let root: RenderResult;
  let mockAddMoreRows: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    mockAddMoreRows = jasmine.createSpy();
    root = render(
      <TaskSummaryReportTable rows={getTaskSummaryLogs()} addMoreRows={mockAddMoreRows} />,
    );
  });

  afterEach(cleanup);

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('task-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(format(timestamp, 'MMM dd yyy hh:mm aaa'));
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(100);
  });

  it('shows the correct headers', () => {
    expect(screen.queryByText('Task ID')).toBeTruthy();
    expect(screen.queryByText('Fleet')).toBeTruthy();
    expect(screen.queryByText('Robot')).toBeTruthy();
    expect(screen.queryByText('Task Description')).toBeTruthy();
    expect(screen.queryByText('State')).toBeTruthy();
    expect(screen.queryByText('Time')).toBeTruthy();
    expect(screen.queryByText('Timestamp')).toBeTruthy();
  });

  it('executes the addMoreRows function', () => {
    const nextPageButton = screen.queryByTitle('Go to next page');
    nextPageButton && userEvent.click(nextPageButton);
    expect(mockAddMoreRows).toHaveBeenCalled();
  });
});
