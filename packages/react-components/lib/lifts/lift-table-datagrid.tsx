import {
  DataGrid,
  GridColDef,
  GridEventListener,
  GridRowParams,
  GridValueGetterParams,
  GridCellParams,
  MuiEvent,
} from '@mui/x-data-grid';
import { Box, SxProps, Typography, useTheme, useMediaQuery } from '@mui/material';
import React from 'react';
import { Lift } from 'api-client';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import { LiftState as LiftStateModel } from 'rmf-models';
import { doorStateToString, liftModeToString, motionStateToString } from './lift-utils';
import { LiftControls } from './lift-controls';

export interface LiftTableData {
  index: number;
  name: string;
  mode: number;
  currentFloor?: string;
  destinationFloor?: string;
  doorState: number;
  motionState: number;
  lift: Lift;
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
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const handleEvent: GridEventListener<'rowClick'> = (
    params: GridRowParams,
    event: MuiEvent<React.MouseEvent<HTMLElement>>,
  ) => {
    if (onLiftClick) {
      onLiftClick(event, params.row);
    }
  };

  const modeState = (params: GridCellParams): React.ReactNode => {
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

      switch (params.row.mode) {
        case LiftStateModel.MODE_AGV:
          return agv;
        case LiftStateModel.MODE_HUMAN:
          return human;
        case LiftStateModel.MODE_FIRE:
        case LiftStateModel.MODE_OFFLINE:
        case LiftStateModel.MODE_EMERGENCY:
          return unstable;
        case LiftStateModel.MODE_UNKNOWN:
        default:
          return unknown;
      }
    })();

    return (
      <Box component="div" sx={modeStateLabelStyle}>
        <Typography
          data-testid="mode-state"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: isScreenHeightLessThan800 ? 10 : 16,
          }}
        >
          {liftModeToString(params.row.mode).toUpperCase()}
        </Typography>
      </Box>
    );
  };

  const LiftState = (params: GridCellParams): React.ReactNode => {
    const currDoorMotion = doorStateToString(params.row?.doorState);
    const currMotion = motionStateToString(params.row?.motionState);

    const motionArrowActiveStyle: SxProps = {
      transform: `scale(${isScreenHeightLessThan800 ? 0.8 : 1})`,
      color: theme.palette.primary.main,
    };

    const motionArrowIdleStyle: SxProps = {
      transform: `scale(${isScreenHeightLessThan800 ? 0.8 : 1})`,
      color: theme.palette.action.disabled,
      opacity: theme.palette.action.disabledOpacity,
    };

    const doorStateLabelStyle: SxProps = (() => {
      switch (params.row?.doorState) {
        case LiftStateModel.DOOR_OPEN:
          return {
            color: theme.palette.success.main,
          };
        case LiftStateModel.DOOR_CLOSED:
          return {
            color: theme.palette.error.main,
          };
        case LiftStateModel.DOOR_MOVING:
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
      <Box component="div" sx={{ display: 'flex', alignItems: 'center', ...doorStateLabelStyle }}>
        <Typography
          component="p"
          sx={{
            marginRight:
              params.row?.doorState === LiftStateModel.DOOR_OPEN
                ? isScreenHeightLessThan800
                  ? 2
                  : 4
                : isScreenHeightLessThan800
                ? 0.4
                : 2,
            fontWeight: 'bold',
            fontSize: isScreenHeightLessThan800 ? 10 : 16,
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
      field: 'mode',
      headerName: 'Mode',
      width: 90,
      flex: 1,
      renderCell: modeState,
      filterable: true,
    },
    {
      field: 'currentFloor',
      headerName: 'Current Floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.currentFloor ? params.row.currentFloor : 'N/A',
      flex: 1,
      filterable: true,
    },
    {
      field: 'destinationFloor',
      headerName: 'Destination Floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.destinationFloor ? params.row.destinationFloor : 'N/A',
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
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight={true}
        getRowId={(l) => l.index}
        rows={lifts}
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
          noRowsLabel: 'No lifts available.',
        }}
        onRowClick={handleEvent}
        initialState={{
          sorting: {
            sortModel: [{ field: 'name', sort: 'asc' }],
          },
        }}
      />
    </div>
  );
}
