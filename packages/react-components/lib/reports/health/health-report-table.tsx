import React from 'react';
import MaterialTable from 'material-table';
import { makeStyles, Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';

export type HealthRowsType = {
  // id: number;
  created: string; //date
  device: string;
  actor_id: string;
  health_status: string;
  health_message: string;
  payload: any;
}[];

export interface HealthReportTable extends DefaultLogTableProps {
  rows: HealthRowsType | [];
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

export const HealthReportTable = (props: HealthReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;
  const classes = useStyles();

  return (
    <MaterialTable
      title="Health"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Device</Typography>,
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
            return <Typography className={classes.cellContent}>{rowData.device}</Typography>;
          },
          // lookup: logLevels as Column<{
          //   level: string;
          //   message: string;
          //   timestamp: string;
          // }>['lookup'],
          // filterComponent: (props) => <CustomLookupFilterParser {...props} />,
        },
        {
          title: <Typography>Actor</Typography>,
          field: 'message',
          type: 'string',
          cellStyle: { padding: '0px', width: '75rem', minWidth: '75rem', whiteSpace: 'pre-wrap' },
          headerStyle: {
            width: '75rem',
            minWidth: '75rem',
          },
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.actor_id}</Typography>;
          },
        },
        {
          title: <Typography>Health status</Typography>,
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
            return <Typography className={classes.cellContent}>{rowData.health_status}</Typography>;
          },
          // lookup: logLevels as Column<{
          //   level: string;
          //   message: string;
          //   timestamp: string;
          // }>['lookup'],
          // filterComponent: (props) => <CustomLookupFilterParser {...props} />,
        },
        {
          title: <Typography>Health message</Typography>,
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
            return <Typography className={classes.cellContent}>{rowData.health_status}</Typography>;
          },
          // lookup: logLevels as Column<{
          //   level: string;
          //   message: string;
          //   timestamp: string;
          // }>['lookup'],
          // filterComponent: (props) => <CustomLookupFilterParser {...props} />,
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
