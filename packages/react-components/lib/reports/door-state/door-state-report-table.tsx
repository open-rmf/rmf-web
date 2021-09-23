import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
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
  const { rows, addMoreRows } = props;
  return (
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight={true}
        getRowId={(r) => r.door.id}
        columns={[
          {
            headerName: 'Name',
            field: 'name',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.door.name}</Typography>;
            },
          },
          {
            headerName: 'State',
            field: 'state',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.state}</Typography>;
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
                <Typography data-testid={'door-table-date'}>
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
