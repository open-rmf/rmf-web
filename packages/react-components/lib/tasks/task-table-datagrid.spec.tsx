import { cleanup, render, RenderResult, screen } from '@testing-library/react';
import React from 'react';
import { TaskDataGridTable, Tasks, FilterFields, SortFields } from './task-table-datagrid';
import { makeTaskQueueEntry } from './test-data.spec';

describe('Tasks table', () => {
  const tasks: Tasks = {
    isLoading: false,
    entries: [],
    requests: {},
    total: 0,
    page: 1,
    pageSize: 10,
  };

  tasks.entries = [makeTaskQueueEntry('task_0'), makeTaskQueueEntry('task_1')];
  let root: RenderResult;
  let mockRowClick: ReturnType<typeof jasmine.createSpy>;
  let mockAddMoreRows: ReturnType<typeof jasmine.createSpy>;

  beforeEach(() => {
    const setFilterFields: FilterFields = {
      model: undefined,
    };

    const setSortFields: SortFields = {
      model: undefined,
    };

    mockAddMoreRows = jasmine.createSpy();
    root = render(
      <TaskDataGridTable
        tasks={tasks}
        onTaskClick={mockRowClick}
        onPageChange={mockAddMoreRows}
        onPageSizeChange={mockAddMoreRows}
        setFilterFields={() => setFilterFields}
        setSortFields={() => setSortFields}
      />,
    );
  });

  afterEach(cleanup);

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(2);
  });

  it('shows titles correctly', () => {
    expect(screen.queryByText('Date')).toBeTruthy();
    expect(screen.queryByText('Requester')).toBeTruthy();
    expect(screen.queryByText('Pickup')).toBeTruthy();
    expect(screen.queryByText('Destination')).toBeTruthy();
    expect(screen.queryByText('Robot')).toBeTruthy();
    expect(screen.queryByText('Start Time')).toBeTruthy();
    expect(screen.queryByText('End Time')).toBeTruthy();
    expect(screen.queryByText('State')).toBeTruthy();
  });
});
