import React from 'react';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type IngestorStateRowsType = {
  created: string; //date
  guid: string;
  state: string;
}[];

export interface IngestorStateReportTable extends DefaultLogTableProps {
  rows: IngestorStateRowsType | [];
}

export const IngestorStateReportTable = (props: IngestorStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <div style={{ height: tableSize, width: '100%' }}>
      <DataGrid
        // title="Ingestor State"
        // icons={materialTableIcons}
        getRowId={(r) => r.guid}
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
            field: 'guid',
            type: 'string',
            valueFormatter: (rowData) => {
              return <Typography>{rowData.row.guid}</Typography>;
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
                <Typography data-testid={'ingestor-table-date'}>
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
