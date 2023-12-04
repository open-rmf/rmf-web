import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridRowParams,
  GridValueGetterParams,
  GridCellParams,
  MuiEvent,
} from '@mui/x-data-grid';
import { Box, Button, SxProps, Typography, useTheme, useMediaQuery } from '@mui/material';
import React from 'react';
import { DoorState } from 'api-client';
import { DoorMode } from 'rmf-models';
import { doorModeToString, doorTypeToString } from './door-utils';
import { HealthStatus, healthStatusToOpMode } from '../utils';

export interface DoorTableData {
  index: number;
  doorName: string;
  opMode: string;
  levelName: string;
  doorType: number;
  doorState?: DoorState;
  onClickOpen?(): void;
  onClickClose?(): void;
}

export interface DoorDataGridTableProps {
  doors: DoorTableData[];
  onDoorClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, doorData: DoorTableData): void;
}

export function DoorDataGridTable({ doors, onDoorClick }: DoorDataGridTableProps): JSX.Element {
  const theme = useTheme();
  const isScreenWidthLessThan1600 = useMediaQuery('(max-width:1600px)');

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onDoorClick) {
      onDoorClick(event, params.row);
    }
  };

  const OpModeState = (params: GridCellParams): React.ReactNode => {
    const opModeStateLabelStyle: SxProps = (() => {
      const unknown = {
        color: theme.palette.action.disabledBackground,
      };
      const online = {
        color: theme.palette.success.main,
      };
      const unstable = {
        color: theme.palette.warning.main,
      };
      const offline = {
        color: theme.palette.error.main,
      };

      switch (params.row.opMode) {
        case HealthStatus.Healthy:
          return online;
        case HealthStatus.Unhealthy:
          return unstable;
        case HealthStatus.Dead:
          return offline;
        default:
          return unknown;
      }
    })();

    return (
      <Box component="div" sx={opModeStateLabelStyle}>
        <Typography
          data-testid="op-mode-state"
          component="p"
          sx={{
            fontWeight: 'bold',
          }}
        >
          {healthStatusToOpMode(params.row.opMode)}
        </Typography>
      </Box>
    );
  };

  const DoorState = (params: GridCellParams): React.ReactNode => {
    const labelStyle: SxProps = React.useMemo<SxProps>(() => {
      const disabled = {
        color: theme.palette.action.disabledBackground,
      };
      const open = {
        color: theme.palette.success.main,
      };
      const closed = {
        color: theme.palette.error.main,
      };
      const moving = {
        color: theme.palette.warning.main,
      };

      switch (params.row.doorState.current_mode.value) {
        case DoorMode.MODE_OPEN:
          return open;
        case DoorMode.MODE_CLOSED:
          return closed;
        case DoorMode.MODE_MOVING:
          return moving;
        default:
          return disabled;
      }
    }, [params.row.doorState.current_mode.value]);

    return (
      <Box component="div" sx={labelStyle}>
        <Typography
          data-testid="door-state"
          component="p"
          sx={{
            fontWeight: 'bold',
          }}
        >
          {params.row.doorState ? doorModeToString(params.row.doorState.current_mode.value) : -1}
        </Typography>
      </Box>
    );
  };

  const OpenCloseButtons = (params: GridCellParams): React.ReactNode => {
    return (
      <Box component="div" sx={{ margin: 0, padding: 0, paddingRight: 1 }}>
        <Button
          variant="contained"
          size="small"
          aria-label="open"
          sx={{
            minWidth: 'auto',
          }}
          onClick={params.row.onClickOpen}
        >
          <Typography>Open</Typography>
        </Button>
        <Button
          variant="contained"
          size="small"
          aria-label="close"
          sx={{
            minWidth: 'auto',
          }}
          onClick={params.row.onClickClose}
        >
          <Typography>Close</Typography>
        </Button>
      </Box>
    );
  };

  const columns: GridColDef[] = [
    {
      field: 'doorName',
      headerName: 'Name',
      width: 90,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.doorName}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'opMode',
      headerName: 'Op. Mode',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: OpModeState,
      filterable: true,
      sortable: false,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'levelName',
      headerName: 'Current Floor',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.levelName}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      sortable: false,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'doorType',
      headerName: 'Type',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{doorTypeToString(params.row.doorType)}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      sortable: false,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'doorState',
      headerName: 'Door State',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: DoorState,
      filterable: true,
      sortable: false,
      headerClassName: 'datagrid-header',
    },
    {
      field: '-',
      headerName: '',
      width: 150,
      editable: false,
      renderCell: OpenCloseButtons,
      flex: 1,
      filterable: false,
      sortable: false,
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
        autoHeight
        getRowId={(l) => l.index}
        rows={doors}
        pageSize={5}
        rowHeight={38}
        columns={columns}
        rowsPerPageOptions={[5]}
        autoPageSize={isScreenWidthLessThan1600}
        density={isScreenWidthLessThan1600 ? 'compact' : 'standard'}
        localeText={{
          noRowsLabel: 'No doors available.',
        }}
        onRowClick={handleEvent}
        initialState={{
          sorting: {
            sortModel: [{ field: 'doorName', sort: 'asc' }],
          },
        }}
        sx={{
          '& .MuiDataGrid-menuIcon': {
            visibility: 'visible',
            width: 'auto',
          },
          overflowX: 'scroll',
        }}
      />
    </Box>
  );
}
