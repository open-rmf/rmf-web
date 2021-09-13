import React from 'react';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type DoorStateRowsType = {
  created: string; //date
  door: { id: number; name: string };
  state: string;
}[];

export interface DoorStateReportTable extends DefaultLogTableProps {
  rows: DoorStateRowsType | [];
}

export const DoorStateReportTable = (props: DoorStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;
  return (
    <div style={{ height: tableSize, width: '100%' }}>
      <DataGrid
        // title="Door State"
        // icons={materialTableIcons}
        columns={[
          {
            headerName: 'Name',
            field: 'name',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.door.name}</Typography>;
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
            headerName: 'Timestamp',
            field: 'created',
            type: 'datetime',
            filterable: false,
            align: 'center',
            valueFormatter: (rowData) => {
              return (
                <Typography data-testid={'door-table-date'}>
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
