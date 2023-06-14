import { DataGrid, GridColDef, GridValueGetterParams, GridCellParams } from '@mui/x-data-grid';
import { Box, SxProps, Typography, useTheme } from '@mui/material';
import * as React from 'react';
import { Lift } from 'api-client';
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward';
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward';
import { LiftState as LiftStateModel } from 'rmf-models';
import { doorStateToString, motionStateToString } from './lift-utils';
import { LiftControls } from './lift-controls';

export interface LiftTableData {
  index: number;
  name: string;
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
}

export function LiftDataGridTable({ lifts }: LiftDataGridTableProps): JSX.Element {
  const theme = useTheme();

  const DoorState = (params: GridCellParams): React.ReactNode => {
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
            marginRight: params.row?.doorState === LiftStateModel.DOOR_OPEN ? 4 : 2,
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
      field: 'doorState',
      headerName: 'Door State',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: DoorState,
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
        localeText={{
          noRowsLabel: 'No lifts available.',
        }}
      />
    </div>
  );
}