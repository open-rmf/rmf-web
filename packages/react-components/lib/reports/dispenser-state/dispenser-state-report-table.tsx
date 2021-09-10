import React from 'react';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type DispenserStateRowsType = {
  created: string; //date
  guid: string;
  state: string;
}[];

export interface DispenserStateReportTable extends DefaultLogTableProps {
  rows: DispenserStateRowsType | [];
}

export const DispenserStateReportTable = (props: DispenserStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <DataGrid
      // title="Dispenser State"
      // icons={materialTableIcons}
      columns={[
        {
          headerName: 'Guid',
          field: 'guid',
          type: 'string',
          valueFormatter: (rowData) => {
            return <Typography>{rowData.row.guid}</Typography>;
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
              <Typography data-testid={'dispenser-table-date'}>
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
  );
};
