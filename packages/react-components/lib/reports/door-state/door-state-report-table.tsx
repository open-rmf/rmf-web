import React from 'react';
import MaterialTable from 'material-table';
import { Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';

export type DoorStateRowsType = {
  created: string; //date
  name: string;
  state: string;
  payload: string | unknown;
}[];

export interface DoorStateReportTable extends DefaultLogTableProps {
  rows: DoorStateRowsType | [];
}

export const DoorStateReportTable = (props: DoorStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <MaterialTable
      title="Door State"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Name</Typography>,
          field: 'name',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.name}</Typography>;
          },
        },
        {
          title: <Typography>State</Typography>,
          field: 'state',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.state}</Typography>;
          },
        },
        {
          title: <Typography>Timestamp</Typography>,
          field: 'created',
          type: 'datetime',
          filtering: false,
          align: 'center',
          render: (rowData) => {
            return (
              <Typography data-testid={'door-table-date'}>
                {moment(rowData.created).format('lll')}
              </Typography>
            );
          },
        },
      ]}
      data={rows}
      options={{
        filtering: true,
        search: false,
        pageSize: 100,
        pageSizeOptions: [50, 100, 200],
        maxBodyHeight: tableSize ? tableSize : '80vh',
      }}
      onChangePage={(page, pageSize) => {
        if (addMoreRows) {
          rows.length / pageSize - 1 === page && addMoreRows();
        }
      }}
    />
  );
};
