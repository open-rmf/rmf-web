import React from 'react';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
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
  const { rows, tableSize, addMoreRows } = props;

  return (
    <div style={{ height: tableSize, width: '100%' }}>
      <DataGrid
        // title="Lift State"
        // icons={materialTableIcons}
        getRowId={(r) => r.client_id}
        columns={[
          {
            headerName: 'Username',
            field: 'username',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.username}</Typography>;
            },
          },

          {
            headerName: 'Client ID',
            field: 'client_id',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.client_id}</Typography>;
            },
          },
          {
            headerName: 'IP Addr.',
            field: 'ip_address',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.ip_address}</Typography>;
            },
          },

          {
            headerName: 'Timestamp',
            field: 'timestamp',
            type: 'datetime',
            filterable: false,
            align: 'center',
            valueFormatter: (rowData) => {
              return (
                <Typography data-testid={'lift-table-date'}>
                  {format(new Date(rowData.row.created), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        rows={rows}
        pageSize={100}
        rowsPerPageOptions={[50, 100, 200]}
        // options={{
        //   filtering: true,
        //   search: false,
        //   pageSize: 100,
        //   pageSizeOptions: [50, 100, 200],
        //   maxBodyHeight: tableSize ? tableSize : '80vh',
        // }}
        onPageChange={(page, pageSize) => {
          if (addMoreRows) {
            rows.length / pageSize - 1 === page && addMoreRows();
          }
        }}
      />
    </div>
  );
};
