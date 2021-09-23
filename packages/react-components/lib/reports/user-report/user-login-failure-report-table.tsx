import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type UserLoginFailureRowsType = {
  created: string; //date
  ip_address: string;
  client_id: string;
  username: string;
  error: string;
}[];

export interface UserLoginFailureReportTable extends DefaultLogTableProps {
  rows: UserLoginFailureRowsType;
}

export const UserLoginFailureReportTable = (
  props: UserLoginFailureReportTable,
): React.ReactElement => {
  const { rows, addMoreRows } = props;

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight={true}
        getRowId={(r) => r.client_id}
        columns={[
          {
            headerName: 'Username',
            field: 'username',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.username}</Typography>;
            },
          },

          {
            headerName: 'Client ID',
            field: 'client_id',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.client_id}</Typography>;
            },
          },
          {
            headerName: 'IP Addr.',
            field: 'ip_address',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.ip_address}</Typography>;
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
                <Typography data-testid={'lift-table-date'}>
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
