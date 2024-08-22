import { Box, Button, SxProps, Typography, useMediaQuery, useTheme } from '@mui/material';
import {
  DataGrid,
  GridCellParams,
  GridColDef,
  GridEventListener,
  GridRowParams,
  GridValueGetterParams,
  MuiEvent,
} from '@mui/x-data-grid';
import { DoorMode, DoorState } from 'api-client';
import React from 'react';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';

import { doorModeToString, doorTypeToString } from './door-utils';

export function doorModeToOpModeString(mode: DoorMode): string {
  switch (mode.value) {
    case RmfDoorMode.MODE_OFFLINE:
      return 'OFFLINE';
    case RmfDoorMode.MODE_UNKNOWN:
      return 'UNKNOWN';
  }
  return 'ONLINE';
}

export interface DoorTableData {
  index: number;
  doorName: string;
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
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const OpModeState = (params: GridCellParams): React.ReactNode => {
    const opModeStateLabelStyle: SxProps = (() => {
      const unknown = {
        color: theme.palette.action.disabledBackground,
      };
      const online = {
        color: theme.palette.success.main,
      };
      const offline = {
        color: theme.palette.error.main,
      };

      if (!params.row.doorState) {
        return unknown;
      }

      switch (params.row.doorState.current_mode.value) {
        case RmfDoorMode.MODE_OFFLINE:
          return offline;
        case RmfDoorMode.MODE_UNKNOWN:
          return unknown;
        default:
          return online;
      }
    })();

    return (
      <Box component="div" sx={opModeStateLabelStyle}>
        <Typography
          data-testid="op-mode-state"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: 14,
          }}
        >
          {doorModeToOpModeString(params.row.doorState.current_mode)}
        </Typography>
      </Box>
    );
  };

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onDoorClick) {
      onDoorClick(event, params.row);
    }
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
        case RmfDoorMode.MODE_OPEN:
          return open;
        case RmfDoorMode.MODE_CLOSED:
          return closed;
        case RmfDoorMode.MODE_MOVING:
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
            fontSize: isScreenHeightLessThan800 ? 10 : 16,
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
            fontSize: isScreenHeightLessThan800 ? 10 : 16,
            marginRight: isScreenHeightLessThan800 ? 0 : 0,
          }}
          onClick={params.row.onClickOpen}
        >
          Open
        </Button>
        <Button
          variant="contained"
          size="small"
          aria-label="close"
          sx={{
            minWidth: 'auto',
            fontSize: isScreenHeightLessThan800 ? 10 : 16,
            marginRight: isScreenHeightLessThan800 ? 0 : 0,
          }}
          onClick={params.row.onClickClose}
        >
          Close
        </Button>
      </Box>
    );
  };

  const columns: GridColDef[] = [
    {
      field: 'doorName',
      headerName: 'Name',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => params.row.doorName,
      flex: 1,
      filterable: true,
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
    },
    {
      field: 'levelName',
      headerName: 'Current Floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.levelName,
      flex: 1,
      filterable: true,
      sortable: false,
    },
    {
      field: 'doorType',
      headerName: 'Type',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => doorTypeToString(params.row.doorType),
      flex: 1,
      filterable: true,
      sortable: false,
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
    },
  ];

  return (
    <DataGrid
      getRowId={(l) => l.index}
      rows={doors}
      pageSize={5}
      rowHeight={38}
      columns={columns}
      rowsPerPageOptions={[5]}
      sx={{
        fontSize: isScreenHeightLessThan800 ? '0.7rem' : 'inherit',
      }}
      autoPageSize={isScreenHeightLessThan800}
      density={isScreenHeightLessThan800 ? 'compact' : 'standard'}
      localeText={{
        noRowsLabel: 'No doors available.',
      }}
      onRowClick={handleEvent}
      initialState={{
        sorting: {
          sortModel: [{ field: 'doorName', sort: 'asc' }],
        },
      }}
      disableVirtualization={true}
    />
  );
}
