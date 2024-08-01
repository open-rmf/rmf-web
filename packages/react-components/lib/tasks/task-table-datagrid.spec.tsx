import { cleanup, render, RenderResult, screen } from '@testing-library/react';

import { FilterFields, TaskDataGridTable, Tasks } from './task-table-datagrid';
import { makeTaskState } from './test-data.spec';

describe('Tasks table', () => {
  const tasks: Tasks = {
    isLoading: false,
    data: [],
    total: 0,
    page: 1,
    pageSize: 10,
  };

  tasks.data = [makeTaskState('task_0'), makeTaskState('task_1')];
  let root: RenderResult;
  let mockAddMoreRows: ReturnType<typeof vi.fn>;

  beforeEach(() => {
    const setFilterFields: FilterFields = {
      model: undefined,
    };

    mockAddMoreRows = vi.fn();
    root = render(
      <TaskDataGridTable
        tasks={tasks}
        onPageChange={mockAddMoreRows}
        onPageSizeChange={mockAddMoreRows}
        setFilterFields={() => setFilterFields}
        setSortFields={() => {}}
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
