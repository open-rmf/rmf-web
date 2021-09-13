import React from 'react';
// import MaterialTable from 'material-table';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
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
  const { rows, tableSize, addMoreRows } = props;

  return (
    <div style={{ height: tableSize, width: '100%' }}>
      <DataGrid
        // title="Health"
        // icons={materialTableIcons}
        columns={[
          {
            headerName: 'Device',
            field: 'device',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.device.type}</Typography>;
            },
          },
          {
            headerName: 'Actor',
            field: 'actor_id',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.device.actor}</Typography>;
            },
          },
          {
            headerName: 'Health Status',
            field: 'health_status',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.health_status}</Typography>;
            },
          },
          {
            headerName: 'Health Message',
            field: 'health_message',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.health_message}</Typography>;
            },
          },
          {
            headerName: 'Timestamp',
            field: 'created',
            type: 'datetime',
            filterable: false,
            align: 'center',
            valueFormatter: (rowData) => {
              return (
                <Typography data-testid={'health-table-date'}>
                  {format(new Date(rowData.row.created), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        rows={rows}
        pageSize={100}
        rowsPerPageOptions={[50, 100, 200]}
        onPageChange={(page, pageSize) => {
          if (addMoreRows) {
            rows.length / pageSize - 1 === page && addMoreRows();
          }
        }}
      />
    </div>
  );
};
