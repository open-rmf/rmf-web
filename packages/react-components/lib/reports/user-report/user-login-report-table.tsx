import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import { Typography } from '@mui/material';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type UserLoginRowsType = {
  client_id: string;
  created: string; //date
  ip_address: string;
  user_id: string;
  username: string;
}[];

export interface UserLoginReportTable extends DefaultLogTableProps {
  rows: UserLoginRowsType;
}

export const UserLoginReportTable = (props: UserLoginReportTable): React.ReactElement => {
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
            headerName: 'Timestamp',
            field: 'created',
            type: 'datetime',
            filterable: false,
            align: 'center',
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography data-testid={'user-logn-table-date'}>
                  {format(new Date(rowData.value as number), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
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
            headerName: 'User ID',
            field: 'user_id',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.user_id}</Typography>;
            },
          },
          {
            headerName: 'IP Addr',
            field: 'ip_address',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.ip_address}</Typography>;
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
