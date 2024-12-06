import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import { Box, SxProps, Typography, useTheme } from '@mui/material';
import {
  DataGrid,
  GridCellParams,
  GridColDef,
  GridEventListener,
  GridRowParams,
  GridValueGetterParams,
  MuiEvent,
} from '@mui/x-data-grid';
import { Lift, LiftState } from 'api-client';
import React from 'react';
import { LiftState as RmfLiftState } from 'rmf-models/ros/rmf_lift_msgs/msg';

import { LiftControls } from './lift-controls';
import { doorStateToString, liftModeToString, motionStateToString } from './lift-utils';

export interface LiftTableData {
  index: number;
  name: string;
  currentFloor?: string;
  destinationFloor?: string;
  doorState: number;
  motionState: number;
  sessionId?: string;
  lift: Lift;
  liftState?: LiftState;
  onRequestSubmit?(
    event: React.FormEvent,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

export interface LiftDataGridTableProps {
  lifts: LiftTableData[];
  onLiftClick?(ev: MuiEvent<React.MouseEvent<HTMLElement>>, liftData: LiftTableData): void;
}

export function LiftDataGridTable({ lifts, onLiftClick }: LiftDataGridTableProps): JSX.Element {
  const theme = useTheme();

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onLiftClick) {
      onLiftClick(event, params.row);
    }
  };

  const ModeState = (params: GridCellParams): React.ReactNode => {
    const modeStateLabelStyle: SxProps = (() => {
      const unknown = {
        color: theme.palette.action.disabledBackground,
      };
      const agv = {
        color: theme.palette.success.main,
      };
      const unstable = {
        color: theme.palette.warning.main,
      };
      const human = {
        color: theme.palette.error.main,
      };

      if (!params.row.liftState) {
        return unknown;
      }

      switch (params.row.liftState.current_mode) {
        case RmfLiftState.MODE_AGV:
          return agv;
        case RmfLiftState.MODE_HUMAN:
          return human;
        case RmfLiftState.MODE_FIRE:
        case RmfLiftState.MODE_OFFLINE:
        case RmfLiftState.MODE_EMERGENCY:
          return unstable;
        case RmfLiftState.MODE_UNKNOWN:
        default:
          return unknown;
      }
    })();

    return (
      <Box sx={modeStateLabelStyle}>
        <Typography
          data-testid="mode-state"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: 16,
          }}
        >
          {liftModeToString(params.row.liftState?.current_mode).toUpperCase()}
        </Typography>
      </Box>
    );
  };

  const LiftState = (params: GridCellParams): React.ReactNode => {
    const currDoorMotion = doorStateToString(params.row?.doorState);
    const currMotion = motionStateToString(params.row?.motionState);

    const motionArrowActiveStyle: SxProps = {
      color: theme.palette.primary.main,
    };

    const motionArrowIdleStyle: SxProps = {
      color: theme.palette.action.disabled,
      opacity: theme.palette.action.disabledOpacity,
    };

    const doorStateLabelStyle: SxProps = (() => {
      switch (params.row?.doorState) {
        case RmfLiftState.DOOR_OPEN:
          return {
            color: theme.palette.success.main,
          };
        case RmfLiftState.DOOR_CLOSED:
          return {
            color: theme.palette.error.main,
          };
        case RmfLiftState.DOOR_MOVING:
          return {
            color: theme.palette.warning.main,
          };
        default:
          return {
            color: theme.palette.action.disabledBackground,
          };
      }
    })();

    return (
      <Box sx={{ display: 'flex', alignItems: 'center', ...doorStateLabelStyle }}>
        <Typography
          component="p"
          sx={{
            marginRight: params.row?.doorState === RmfLiftState.DOOR_OPEN ? 4 : 2,
            fontWeight: 'bold',
            fontSize: 16,
            display: 'inline-block',
          }}
        >
          {currDoorMotion}
        </Typography>
        <ArrowUpwardIcon sx={currMotion === 'Up' ? motionArrowActiveStyle : motionArrowIdleStyle} />
        <ArrowDownwardIcon
          sx={currMotion === 'Down' ? motionArrowActiveStyle : motionArrowIdleStyle}
        />
      </Box>
    );
  };

  const LiftControl = (params: GridCellParams): React.ReactNode => {
    return (
      <LiftControls
        availableLevels={params.row.lift.levels}
        currentLevel={params.row?.currentFloor}
        onRequestSubmit={params.row?.onRequestSubmit}
      />
    );
  };

  const columns: GridColDef[] = [
    {
      field: 'name',
      headerName: 'Name',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => params.row.name,
      flex: 1,
      filterable: true,
    },
    {
      field: 'opMode',
      headerName: 'Op. Mode',
      width: 90,
      flex: 1,
      renderCell: ModeState,
      filterable: true,
    },
    {
      field: 'currentFloor',
      headerName: 'Current Floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.currentFloor ? params.row.currentFloor : 'n/a',
      flex: 1,
      filterable: true,
    },
    {
      field: 'destinationFloor',
      headerName: 'Destination Floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.destinationFloor ? params.row.destinationFloor : 'n/a',
      flex: 1,
      filterable: true,
    },
    {
      field: 'liftState',
      headerName: 'Lift State',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: LiftState,
      filterable: true,
    },
    {
      field: '-',
      headerName: '',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: LiftControl,
      filterable: true,
      sortable: false,
    },
  ];

  return (
    <DataGrid
      getRowId={(l) => l.index}
      rows={lifts}
      pageSize={5}
      rowHeight={38}
      columns={columns}
      rowsPerPageOptions={[5]}
      density={'standard'}
      localeText={{
        noRowsLabel: 'No lifts available.',
      }}
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
