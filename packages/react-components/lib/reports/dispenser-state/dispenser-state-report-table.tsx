import React from 'react';
import MaterialTable from 'material-table';
import { Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';

export type DispenserStateRowsType = {
  created: string; //date
  guid: string;
  state: string;
  payload: string | unknown;
}[];

export interface DispenserStateReportTable extends DefaultLogTableProps {
  rows: DispenserStateRowsType | [];
}

export const DispenserStateReportTable = (props: DispenserStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <MaterialTable
      title="Dispenser State"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Guid</Typography>,
          field: 'guid',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.guid}</Typography>;
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
              <Typography data-testid={'dispenser-table-date'}>
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
