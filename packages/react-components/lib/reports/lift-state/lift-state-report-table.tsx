import React from 'react';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type LiftStateRowsType = {
  created: string; //date
  state: string;
  lift?: { id: number; name: string };
  door_state: string;
  destination_floor: string;
  motion_state: string;
  current_floor: string;
  session_id: string;
}[];

export interface LiftStateReportTable extends DefaultLogTableProps {
  rows: LiftStateRowsType | [];
}

export const LiftStateReportTable = (props: LiftStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <div style={{ height: tableSize, width: '100%' }}>
      <DataGrid
        // title="Lift State"
        // icons={materialTableIcons}
        getRowId={(r) => r.lift.id}
        columns={[
          {
            headerName: 'Session ID',
            field: 'session_id',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.session_id}</Typography>;
            },
          },
          {
            headerName: 'State',
            field: 'state',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.state}</Typography>;
            },
          },
          {
            headerName: 'Door State',
            field: 'door_state',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.door_state}</Typography>;
            },
          },
          {
            headerName: 'Destination Floor',
            field: 'destination_floor',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.destination_floor}</Typography>;
            },
          },
          {
            headerName: 'Motion State',
            field: 'motion_state',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.motion_state}</Typography>;
            },
          },
          {
            headerName: 'Current Floor',
            field: 'current_floor',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.current_floor}</Typography>;
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
        onPageChange={(page, pageSize) => {
          if (addMoreRows) {
            rows.length / pageSize - 1 === page && addMoreRows();
          }
        }}
      />
    </div>
  );
};
