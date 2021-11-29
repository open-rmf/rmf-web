import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import { Typography } from '@mui/material';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type HealthRowsType = {
  created: string; //date
  device: { id?: number; type: string; actor: string };
  health_status: string;
  health_message: string;
}[];

export interface HealthReportTable extends DefaultLogTableProps {
  rows: HealthRowsType | [];
}

export const HealthReportTable = (props: HealthReportTable): React.ReactElement => {
  const { rows, addMoreRows } = props;

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight={true}
        getRowId={(r) => r.device.id}
        columns={[
          {
            headerName: 'Device',
            field: 'device',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.device.type}</Typography>;
            },
          },
          {
            headerName: 'Actor',
            field: 'actor_id',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.device.actor}</Typography>;
            },
          },
          {
            headerName: 'Health Status',
            field: 'health_status',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.health_status}</Typography>;
            },
          },
          {
            headerName: 'Health Message',
            field: 'health_message',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.health_message}</Typography>;
            },
          },
          {
            headerName: 'Timestamp',
            field: 'created',
            type: 'datetime',
            filterable: false,
            align: 'center',
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography data-testid={'health-table-date'}>
                  {format(new Date(rowData.value as number), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        rows={rows}
        pageSize={100}
        rowsPerPageOptions={[50, 100]}
        onPageChange={() => {
          if (addMoreRows) {
            addMoreRows();
          }
        }}
        disableColumnMenu={true}
      />
    </div>
  );
};
