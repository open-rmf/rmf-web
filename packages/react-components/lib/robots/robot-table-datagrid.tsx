import {
  DataGrid,
  GridColDef,
  GridEventListener,
  MuiEvent,
  GridRowParams,
  GridCellParams,
} from '@mui/x-data-grid';
import { Box, SxProps, Typography, useTheme, useMediaQuery } from '@mui/material';
import * as React from 'react';
import { Status2 } from 'api-client';
import { RobotTableData } from './robot-table';
import { robotStatusToUpperCase } from './utils';

export interface RobotDataGridTableProps {
  onRobotClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, robotName: RobotTableData): void;
  robots: RobotTableData[];
}

export function RobotDataGridTable({ onRobotClick, robots }: RobotDataGridTableProps): JSX.Element {
  const isScreenWidthLessThan1600 = useMediaQuery('(max-width:1600px)');

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onRobotClick) {
      onRobotClick(event, params.row);
    }
  };

  const Status = (params: GridCellParams): React.ReactNode => {
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
      const defaultColor = {
        color: theme.palette.warning.main,
      };

      switch (params.row.status) {
        case Status2.Error:
          return error;
        case Status2.Charging:
          return charging;
        case Status2.Working:
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
          }}
        >
          {robotStatusToUpperCase(params.row.status)}
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
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.name}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'fleet',
      headerName: 'Fleet',
      width: 90,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.fleet}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'estFinishTime',
      headerName: 'Est. Task Finish Time',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>
            {params.row.estFinishTime ? new Date(params.row.estFinishTime).toLocaleString() : '-'}
          </Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'level',
      headerName: 'Level',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.level}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'battery',
      headerName: 'Battery',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{`${(params.row.battery * 100).toFixed(2)}%`}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'lastUpdateTime',
      headerName: 'Last Updated',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>
            {params.row.lastUpdateTime ? new Date(params.row.lastUpdateTime).toLocaleString() : '-'}
          </Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'status',
      headerName: 'Status',
      editable: false,
      flex: 1,
      renderCell: Status,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
  ];

  return (
    <Box
      component="div"
      sx={{
        '& .datagrid-header': {
          fontSize: isScreenWidthLessThan1600 ? '0.7rem' : 'inherit',
        },
      }}
    >
      <DataGrid
        autoHeight={true}
        getRowId={(r) => r.name}
        rows={robots}
        pageSize={5}
        rowHeight={38}
        columns={columns}
        rowsPerPageOptions={[5]}
        autoPageSize={isScreenWidthLessThan1600}
        density={isScreenWidthLessThan1600 ? 'compact' : 'standard'}
        onRowClick={handleEvent}
        initialState={{
          sorting: {
            sortModel: [{ field: 'name', sort: 'asc' }],
          },
        }}
      />
    </Box>
  );
}
