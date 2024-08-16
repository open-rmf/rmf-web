import { useMediaQuery } from '@mui/material';
import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridRowParams,
  GridValueGetterParams,
  MuiEvent,
} from '@mui/x-data-grid';
import * as React from 'react';

export interface MutexGroupData {
  name: string;
  lockedBy?: string;
  requestedBy: string[];
}

export interface MutexGroupTableProps {
  onMutexGroupClick?(
    ev: MuiEvent<React.MouseEvent<HTMLElement>>,
    mutexGroups: MutexGroupData,
  ): void;
  mutexGroups: MutexGroupData[];
}

export function MutexGroupTable({
  onMutexGroupClick,
  mutexGroups,
}: MutexGroupTableProps): JSX.Element {
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onMutexGroupClick) {
      onMutexGroupClick(event, params.row);
    }
  };

  const columns: GridColDef[] = [
    {
      field: 'mutexGroup',
      headerName: 'Group',
      maxWidth: 240,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.name,
      flex: 1,
      filterable: true,
    },
    {
      field: 'lockedBy',
      headerName: 'Locked',
      maxWidth: 240,
      valueGetter: (params: GridValueGetterParams) => params.row.lockedBy ?? 'n/a',
      flex: 1,
      filterable: true,
    },
    {
      field: 'requestedBy',
      headerName: 'Waiting',
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.requestedBy?.join(', ') ?? 'n/a',
      flex: 1,
      filterable: true,
    },
  ];

  return (
    <DataGrid
      getRowId={(r) => r.name}
      rows={mutexGroups}
      pageSize={5}
      rowHeight={38}
      columns={columns}
      rowsPerPageOptions={[5]}
      sx={{
        fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
      }}
      autoPageSize={isScreenHeightLessThan800}
      density={isScreenHeightLessThan800 ? 'compact' : 'standard'}
      onRowClick={handleEvent}
      initialState={{
        sorting: {
          sortModel: [{ field: 'name', sort: 'asc' }],
        },
      }}
      disableVirtualization
    />
  );
}
