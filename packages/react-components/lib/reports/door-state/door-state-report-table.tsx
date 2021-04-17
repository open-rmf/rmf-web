import React from 'react';
import MaterialTable from 'material-table';
import { makeStyles, Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';

export type DoorStateRowsType = {
  // id: number;
  created: string; //date
  name: string;
  state: string;
  payload: any;
}[];

export interface DoorStateReportTable extends DefaultLogTableProps {
  rows: DoorStateRowsType | [];
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

export const DoorStateReportTable = (props: DoorStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;
  const classes = useStyles();

  return (
    <MaterialTable
      title="Door State"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Name</Typography>,
          field: 'level',
          type: 'string',
          align: 'center',
          cellStyle: { padding: '0px', width: '2rem', maxWidth: '2rem' },
          headerStyle: {
            width: '2rem',
            maxWidth: '2rem',
          },
          filterCellStyle: {
            maxHeight: '2px',
          },
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.name}</Typography>;
          },
          // lookup: logLevels as Column<{
          //   level: string;
          //   message: string;
          //   timestamp: string;
          // }>['lookup'],
          // filterComponent: (props) => <CustomLookupFilterParser {...props} />,
        },
        {
          title: <Typography>State</Typography>,
          field: 'message',
          type: 'string',
          cellStyle: { padding: '0px', width: '75rem', minWidth: '75rem', whiteSpace: 'pre-wrap' },
          headerStyle: {
            width: '75rem',
            minWidth: '75rem',
          },
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
          cellStyle: { padding: '0px' },
          render: (rowData) => {
            return (
              <Typography className={classes.cellContent} data-testid={'dispenser-table-date'}>
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
        rows.length / pageSize - 1 === page && addMoreRows();
      }}
    />
  );
};
