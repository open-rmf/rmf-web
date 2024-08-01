import { Box, SxProps, Typography, useMediaQuery, useTheme } from '@mui/material';
import { DataGrid, GridCellParams, GridColDef, GridValueGetterParams } from '@mui/x-data-grid';
import { BeaconState } from 'api-client';
import React from 'react';

export interface BeaconDataGridTableProps {
  beacons: BeaconState[];
}

export function BeaconDataGridTable({ beacons }: BeaconDataGridTableProps): JSX.Element {
  const theme = useTheme();
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  const OpModeState = (params: GridCellParams): React.ReactNode => {
    const opModeStateLabelStyle: SxProps = (() => {
      const online = {
        color: theme.palette.success.main,
      };
      const offline = {
        color: theme.palette.error.main,
      };

      return params.row.online ? online : offline;
    })();

    return (
      <Box component="div" sx={opModeStateLabelStyle}>
        <Typography
          data-testid="op-mode-state"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: isScreenHeightLessThan800 ? 12 : 16,
          }}
        >
          {params.row.online ? 'ONLINE' : 'OFFLINE'}
        </Typography>
      </Box>
    );
  };

  const ActivatedState = (params: GridCellParams): React.ReactNode => {
    const activatedStateLabelStyle: SxProps = (() => {
      const on = {
        color: theme.palette.success.main,
      };
      const off = {
        color: theme.palette.error.main,
      };

      return params.row.activated ? on : off;
    })();

    return (
      <Box component="div" sx={activatedStateLabelStyle}>
        <Typography
          data-testid="activated-state"
          component="p"
          sx={{
            fontWeight: 'bold',
            fontSize: 16,
          }}
        >
          {params.row.activated ? 'ON' : 'OFF'}
        </Typography>
      </Box>
    );
  };

  const columns: GridColDef[] = [
    {
      field: 'beaconName',
      headerName: 'Name',
      width: 90,
      valueGetter: (params: GridValueGetterParams) => params.row.id,
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
    },
    {
      field: 'levelName',
      headerName: 'Level',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.level ?? 'N/A',
      flex: 1,
      filterable: true,
    },
    {
      field: 'beaconCategory',
      headerName: 'Type',
      width: 150,
      editable: false,
      valueGetter: (params: GridValueGetterParams) => params.row.category ?? 'N/A',
      flex: 1,
      filterable: true,
    },
    {
      field: 'beaconState',
      headerName: 'Beacon State',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: ActivatedState,
      filterable: true,
    },
  ];

  return (
    <DataGrid
      autoHeight={true}
      getRowId={(l) => l.id}
      rows={beacons}
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
        noRowsLabel: 'No beacons available.',
      }}
      disableVirtualization={true}
    />
  );
}
