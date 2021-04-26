import { screen, render, RenderResult, cleanup } from '@testing-library/react';
import React from 'react';
import moment from 'moment';
import { HealthReportTable } from '../../../lib';
import { getHealthLogs } from '../utils';
import userEvent from '@testing-library/user-event';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString();

describe('Health table test', () => {
  let root: RenderResult;
  let mockAddMoreRows: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    mockAddMoreRows = jasmine.createSpy();
    root = render(<HealthReportTable rows={getHealthLogs()} addMoreRows={mockAddMoreRows} />);
  });

  afterEach(cleanup);

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('health-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(moment(timestamp).format('lll'));
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('tr').length;
    // -3. from the tr of the table header, filter and pagination table
    expect(allRows - 3).toBe(100);
  });

  it('shows titles correctly', () => {
    expect(screen.queryByText('Device')).toBeTruthy();
    expect(screen.queryByText('Actor')).toBeTruthy();
    expect(screen.queryByText('Health Status')).toBeTruthy();
    expect(screen.queryByText('Health Message')).toBeTruthy();
    expect(screen.queryByText('Timestamp')).toBeTruthy();
  });

  it('executes the addMoreRows', () => {
    const nextPageButton = screen.queryByTitle('Next Page')?.children[0];
    nextPageButton && userEvent.click(nextPageButton);
    expect(mockAddMoreRows).toHaveBeenCalled();
  });
});
