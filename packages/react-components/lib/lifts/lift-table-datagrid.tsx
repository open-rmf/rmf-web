import { DataGrid, GridColDef, GridValueGetterParams, GridCellParams } from '@mui/x-data-grid';
import { Box, SxProps, Typography, styled, useTheme } from '@mui/material';
import * as React from 'react';
import { Lift } from 'api-client';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import { LiftState as LiftStateModel } from 'rmf-models';
import { doorStateToString, motionStateToString } from './lift-utils';
import { LiftControls } from './lift-controls';

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

export interface LiftData {
  index: number;
  name: string;
  current_floor?: string;
  destination_floor?: string;
  door_state: number;
  motion_state: number;
  lift: Lift;
  onRequestSubmit?(
    event: React.FormEvent,
    doorState: number,
    requestType: number,
    destination: string,
  ): void;
}

export interface LiftDataGridTableProps {
  lifts: LiftData[];
}

export function LiftDataGridTable({ lifts }: LiftDataGridTableProps): JSX.Element {
  const theme = useTheme();

  const renderDoorState = (params: GridCellParams): React.ReactNode => {
    const currDoorMotion = doorStateToString(params.row?.door_state);
    const currMotion = motionStateToString(params.row?.motion_state);
    const motionArrowActiveStyle: SxProps = {
      color: theme.palette.primary.main,
    };
    const motionArrowIdleStyle: SxProps = {
      color: theme.palette.action.disabled,
      opacity: theme.palette.action.disabledOpacity,
    };

    const doorStateLabelStyle: SxProps = (() => {
      switch (params.row?.door_state) {
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
      <Box sx={doorStateLabelStyle}>
        <Typography
          component="p"
          sx={{
            marginRight: params.row?.door_state === LiftStateModel.DOOR_OPEN ? 4 : 2,
            fontWeight: 'bold',
            fontSize: 14,
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

  const renderLiftControl = (params: GridCellParams): React.ReactNode => {
    return (
      <LiftControls
        availableLevels={params.row.lift.levels}
        currentLevel={params.row?.current_floor}
        onRequestSubmit={params.row?.test}
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
      field: 'current_floor',
      headerName: 'Current floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.current_floor ? params.row.current_floor : 'N/A',
      flex: 1,
      filterable: true,
    },
    {
      field: 'destination_floor',
      headerName: 'Destination floor',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) =>
        params.row.destination_floor ? params.row.destination_floor : 'N/A',
      flex: 1,
      filterable: true,
    },
    {
      field: 'door_state',
      headerName: 'Door state',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: renderDoorState,
      filterable: true,
    },
    {
      field: '-',
      headerName: '',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: renderLiftControl,
      filterable: true,
    },
  ];

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <StyledDataGrid
        autoHeight={true}
        getRowId={(l) => l.index}
        rows={lifts}
        pageSize={5}
        rowHeight={38}
        columns={columns}
        rowsPerPageOptions={[5]}
      />
    </div>
  );
}
