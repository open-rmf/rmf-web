import { cleanup, render, RenderResult, screen } from '@testing-library/react';
import React from 'react';
import { TaskDataGridTable, DefaultTableDataGridProps } from './task-datagrid-table';
import { makeTaskState } from './test-data.spec';

describe('Tasks table', () => {
  const tasksProps: DefaultTableDataGridProps = {
    isLoading: false,
    data: [],
    total: 0,
    page: 1,
    pageSize: 10,
  };

  tasksProps.data = [makeTaskState('task_0'), makeTaskState('task_1')];
  let root: RenderResult;
  let mockAddMoreRows: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    mockAddMoreRows = jasmine.createSpy();
    root = render(
      <TaskDataGridTable
        tasks={tasksProps}
        onPageChange={mockAddMoreRows}
        onPageSizeChange={mockAddMoreRows}
      />,
    );
  });

  afterEach(cleanup);

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(2);
  });

  it('shows titles correctly', () => {
    expect(screen.queryByText('ID')).toBeTruthy();
    expect(screen.queryByText('Category')).toBeTruthy();
    expect(screen.queryByText('Assignee')).toBeTruthy();
    expect(screen.queryByText('Start Time')).toBeTruthy();
    expect(screen.queryByText('End Time')).toBeTruthy();
    expect(screen.queryByText('State')).toBeTruthy();
  });
});
