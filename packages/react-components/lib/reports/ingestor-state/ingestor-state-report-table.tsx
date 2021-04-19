import React from 'react';
import MaterialTable from 'material-table';
import { makeStyles, Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';

export type IngestorStateRowsType = {
  // id: number;
  created: string; //date
  guid: string;
  state: string;
  payload: any;
}[];

export interface IngestorStateReportTable extends DefaultLogTableProps {
  rows: IngestorStateRowsType | [];
}

const useStyles = makeStyles((theme) => ({
  cellContent: {
    display: 'block',
    marginBlockStart: '1em',
    marginBlockEnd: '1em',
    marginInlineStart: '0px',
    marginInlineEnd: '0px',
  },
}));

export const IngestorStateReportTable = (props: IngestorStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;
  const classes = useStyles();

  return (
    <MaterialTable
      title="Ingestor State"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Guid</Typography>,
          field: 'level',
          type: 'string',
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.guid}</Typography>;
          },
        },
        {
          title: <Typography>State</Typography>,
          field: 'message',
          type: 'string',
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.state}</Typography>;
          },
        },
        {
          title: <Typography>Timestamp</Typography>,
          field: 'timestamp',
          type: 'datetime',
          filtering: false,
          align: 'center',
          render: (rowData) => {
            return (
              <Typography className={classes.cellContent} data-testid={'ingestor-table-date'}>
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
