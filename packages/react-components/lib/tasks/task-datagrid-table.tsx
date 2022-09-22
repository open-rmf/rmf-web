import { DataGrid, GridColDef, GridValueGetterParams } from '@mui/x-data-grid';
import { TaskState } from 'api-client';
import React from 'react';

export interface DefaultTableDataGridProps {
  isLoading: boolean;
  data: TaskState[];
  total: number;
  page: number;
  pageSize: number;
}

export interface TaskDataGridTableProps {
  tasks: DefaultTableDataGridProps;
  onTaskClick?(ev: React.MouseEvent<HTMLDivElement>, task: TaskState): void;
  onPageChange: (newPage: number) => void;
  onPageSizeChange: (newPageSize: number) => void;
}

const columns: GridColDef[] = [
  {
    field: 'id',
    headerName: 'ID',
    width: 90,
    valueGetter: (params: GridValueGetterParams) => params.row.booking.id,
  },
  {
    field: 'category',
    headerName: 'Category',
    width: 150,
    editable: false,
  },
  {
    field: 'name',
    headerName: 'Assignee',
    width: 150,
    editable: true,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.assigned_to ? params.row.assigned_to.name : 'unknown',
  },
  {
    field: 'status',
    headerName: 'State',
    width: 150,
    editable: true,
    valueGetter: (params: GridValueGetterParams) =>
      params.row.status ? params.row.status : 'unknown',
  },
];

export function TaskDataGridTable({
  tasks,
  onTaskClick,
  onPageChange,
  onPageSizeChange,
}: TaskDataGridTableProps): JSX.Element {
  return (
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight
        getRowId={(r) => r.booking.id}
        rows={tasks.data}
        rowCount={tasks.total}
        loading={tasks.isLoading}
        rowsPerPageOptions={[10, 30, 50, 70, 100]}
        pagination
        page={tasks.page - 1}
        pageSize={tasks.pageSize}
        onPageChange={onPageChange}
        onPageSizeChange={onPageSizeChange}
        columns={columns}
      />
    </div>
  );
}
