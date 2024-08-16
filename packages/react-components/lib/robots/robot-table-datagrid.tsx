import { Box, SxProps, Typography, useMediaQuery, useTheme } from '@mui/material';
import {
  DataGrid,
  GridCellParams,
  GridColDef,
  GridEventListener,
  GridRowParams,
  GridValueGetterParams,
  MuiEvent,
} from '@mui/x-data-grid';
import { ApiServerModelsRmfApiRobotStateStatus as Status, Commission } from 'api-client';
import * as React from 'react';

import { robotStatusToUpperCase } from './utils';

export interface RobotTableData {
  fleet: string;
  name: string;
  status?: Status;
  battery?: number;
  estFinishTime?: number;
  lastUpdateTime?: number;
  level?: string;
  commission?: Commission;
}

export interface RobotDataGridTableProps {
  onRobotClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, robotName: RobotTableData): void;
  robots: RobotTableData[];
}

export function RobotDataGridTable({ onRobotClick, robots }: RobotDataGridTableProps): JSX.Element {
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onRobotClick) {
      onRobotClick(event, params.row);
    }
  };

  const StatusCell = (params: GridCellParams): React.ReactNode => {
    const robotDecommissioned =
      params.row.commission && params.row.commission.dispatch_tasks === false;

    const theme = useTheme();
    const statusLabelStyle: SxProps = (() => {
      const error = {
        color: theme.palette.error.main,
      };
      const charging = {
        color: theme.palette.info.main,
      };
      const working = {
        color: theme.palette.success.main,
      };
      const disabled = {
        color: theme.palette.action.disabled,
      };
      const defaultColor = {
        color: theme.palette.warning.main,
      };

      if (robotDecommissioned) {
        return disabled;
      }

      switch (params.row.status) {
        case Status.Error:
          return error;
        case Status.Offline:
        case Status.Uninitialized:
          return disabled;
        case Status.Charging:
          return charging;
        case Status.Working:
          return working;
        default:
          return defaultColor;
      }
    })();

    return (
      <Box component="div" sx={statusLabelStyle}>
        <Typography
          data-testid="status"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: isScreenHeightLessThan800 ? 10 : 16,
          }}
        >
          {robotDecommissioned ? 'DECOMMISSIONED' : robotStatusToUpperCase(params.row.status)}
        </Typography>
      </Box>
    );
  };

  const columns: GridColDef[] = [
    {
      field: 'name',
      headerName: 'Name',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.name,
      flex: 1,
      filterable: true,
    },
    {
      field: 'fleet',
      headerName: 'Fleet',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => params.row.fleet,
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
      field: 'level',
      headerName: 'Level',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.level,
      flex: 1,
      filterable: true,
    },
    {
      field: 'battery',
      headerName: 'Battery',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => `${(params.row.battery * 100).toFixed(2)}%`,
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
      renderCell: StatusCell,
      filterable: true,
    },
  ];

  return (
    <DataGrid
      getRowId={(r) => r.name}
      rows={robots}
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
      disableVirtualization={true}
    />
  );
}
