import { ApiServerModelsTortoiseModelsBeaconsBeaconStateLeaf as BeaconState } from 'api-client';
import { DataGrid, GridColDef, GridValueGetterParams, GridCellParams } from '@mui/x-data-grid';
import { Box, SxProps, Typography, useTheme, useMediaQuery } from '@mui/material';
import React from 'react';

export interface BeaconDataGridTableProps {
  beacons: BeaconState[];
}

export function BeaconDataGridTable({ beacons }: BeaconDataGridTableProps): JSX.Element {
  const theme = useTheme();
  const isScreenWidthLessThan1600 = useMediaQuery('(max-width:1600px)');

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
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.id}</Typography>
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
      headerClassName: 'datagrid-header',
    },
    {
      field: 'levelName',
      headerName: 'Level',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.level ?? 'n/a'}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'beaconCategory',
      headerName: 'Type',
      width: 150,
      editable: false,
      renderCell: (params: GridCellParams): React.ReactNode => (
        <Box component="div">
          <Typography>{params.row.category ?? 'n/a'}</Typography>
        </Box>
      ),
      flex: 1,
      filterable: true,
      headerClassName: 'datagrid-header',
    },
    {
      field: 'beaconState',
      headerName: 'Beacon State',
      width: 150,
      editable: false,
      flex: 1,
      renderCell: ActivatedState,
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
        getRowId={(l) => l.id}
        rows={beacons}
        pageSize={5}
        rowHeight={38}
        columns={columns}
        rowsPerPageOptions={[5]}
        autoPageSize={isScreenWidthLessThan1600}
        density={isScreenWidthLessThan1600 ? 'compact' : 'standard'}
        localeText={{
          noRowsLabel: 'No beacons available.',
        }}
        sx={{
          '& .MuiDataGrid-menuIcon': {
            visibility: 'visible',
            width: 'auto',
          },
        }}
      />
    </Box>
  );
}
