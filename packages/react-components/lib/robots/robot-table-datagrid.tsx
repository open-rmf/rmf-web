import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridValueGetterParams,
  MuiEvent,
  GridRowParams,
  GridCellParams,
} from '@mui/x-data-grid';
import { styled } from '@mui/material';
import * as React from 'react';
import { Status2 } from 'api-client';
import { RobotTableData } from './robot-table';

const classes = {
  robotErrorCell: 'MuiDataGrid-cell-error-cell',
  robotChargingCell: 'MuiDataGrid-cell-charging-cell',
  robotWorkingCell: 'MuiDataGrid-cell-working-cell',
  robotIdleCell: 'MuiDataGrid-cell-idle-cell',
  robotOfflineCell: 'MuiDataGrid-cell-offline-cell',
  robotShutdownCell: 'MuiDataGrid-cell-shutdown-cell',
  robotDefaultCell: 'MuiDataGrid-cell-defautl-cell',
};

const StyledDataGrid = styled(DataGrid)(({ theme }) => ({
  [`& .${classes.robotErrorCell}`]: {
    backgroundColor: theme.palette.error.main,
    color: theme.palette.getContrastText(theme.palette.success.light),
  },
  [`& .${classes.robotChargingCell}`]: {
    backgroundColor: theme.palette.info.main,
    color: theme.palette.getContrastText(theme.palette.grey[500]),
  },
  [`& .${classes.robotWorkingCell}`]: {
    backgroundColor: theme.palette.success.main,
    color: theme.palette.getContrastText(theme.palette.info.light),
  },
  [`& .${classes.robotDefaultCell}`]: {
    backgroundColor: theme.palette.warning.main,
    color: theme.palette.getContrastText(theme.palette.warning.main),
  },
}));

export interface RobotDataGridTableProps {
  onRobotClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, robotName: RobotTableData): void;
  robots: RobotTableData[];
}

export function RobotDataGridTable({ onRobotClick, robots }: RobotDataGridTableProps): JSX.Element {
  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onRobotClick) {
      onRobotClick(event, params.row);
    }
  };

  const columns: GridColDef[] = [
    {
      field: 'fleet',
      headerName: 'Fleet',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => params.row.fleet,
      flex: 1,
      filterable: true,
    },
    {
      field: 'name',
      headerName: 'Robot Name',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.name,
      flex: 1,
      filterable: true,
    },
    {
      field: 'estFinishTime',
      headerName: 'Est. Task Finish Time',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.estFinishTime ? new Date(params.row.estFinishTime).toLocaleString() : '-',
      flex: 1,
      filterable: true,
    },
    {
      field: 'battery',
      headerName: 'Battery',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => (params.row.battery * 100).toFixed(2),
      flex: 1,
      filterable: true,
    },
    {
      field: 'lastUpdateTime',
      headerName: 'Last Updated',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.lastUpdateTime ? new Date(params.row.lastUpdateTime).toLocaleString() : '-',
      flex: 1,
      filterable: true,
    },
    {
      field: 'status',
      headerName: 'Status',
      editable: false,
      flex: 1,
      filterable: true,
    },
  ];

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <StyledDataGrid
        autoHeight={true}
        getRowId={(r) => r.name}
        rows={robots}
        pageSize={5}
        rowHeight={38}
        columns={columns}
        rowsPerPageOptions={[5]}
        onRowClick={handleEvent}
        getCellClassName={(params: GridCellParams<string>) => {
          if (params.field === 'status') {
            switch (params.value) {
              case Status2.Error:
                return classes.robotErrorCell;
              case Status2.Charging:
                return classes.robotChargingCell;
              case Status2.Working:
                return classes.robotWorkingCell;
              case Status2.Idle:
              case Status2.Offline:
              case Status2.Shutdown:
              case Status2.Uninitialized:
                return classes.robotDefaultCell;
              default:
                return classes.robotDefaultCell;
            }
          }
          return '';
        }}
      />
    </div>
  );
}
