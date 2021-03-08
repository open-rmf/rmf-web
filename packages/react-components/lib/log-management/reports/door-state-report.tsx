import React from 'react';
import MaterialTable from 'material-table';
import { makeStyles, Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';

export type DoorRowsType = { name: string; status: string; message: string; timestamp: string }[];

export interface DoorTableProps {
  rows: DoorRowsType | [];
  tableSize?: string; // units vh or rem
}

const useStyles = makeStyles(() => ({
  cellContent: {
    display: 'block',
    marginBlockStart: '1em',
    marginBlockEnd: '1em',
    marginInlineStart: '0px',
    marginInlineEnd: '0px',
  },
}));

export const DoorStateReport = (props: DoorTableProps): React.ReactElement => {
  const { rows, tableSize } = props;
  const classes = useStyles();

  return (
    <MaterialTable
      title="Door States"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Name</Typography>,
          field: 'name',
          type: 'string',
        },
        {
          title: <Typography>Status</Typography>,
          field: 'status',
          type: 'string',
        },
        {
          title: <Typography>Message</Typography>,
          field: 'message',
          type: 'string',
        },
        {
          title: <Typography>Timestamp</Typography>,
          field: 'timestamp',
          type: 'datetime',
          filtering: false,
          align: 'center',
          cellStyle: { padding: '0px' },
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
    />
  );
};
