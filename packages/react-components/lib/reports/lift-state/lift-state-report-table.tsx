import React from 'react';
import MaterialTable from 'material-table';
import { makeStyles, Typography } from '@material-ui/core';
import moment from 'moment';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';

export type LiftStateRowsType = {
  created: string; //date
  state: string;
  door_state: string;
  destination_floor: string;
  motion_state: string;
  current_floor: string;
  session_id: string;
  payload: any;
}[];

export interface LiftStateReportTable extends DefaultLogTableProps {
  rows: LiftStateRowsType | [];
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

export const LiftStateReportTable = (props: LiftStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;
  const classes = useStyles();

  return (
    <MaterialTable
      title="Lift State"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Session ID</Typography>,
          field: 'level',
          type: 'string',
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.session_id}</Typography>;
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
          title: <Typography>Door State</Typography>,
          field: 'message',
          type: 'string',
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.door_state}</Typography>;
          },
        },
        {
          title: <Typography>Destination Floor</Typography>,
          field: 'message',
          type: 'string',
          render: (rowData) => {
            return (
              <Typography className={classes.cellContent}>{rowData.destination_floor}</Typography>
            );
          },
        },
        {
          title: <Typography>Motion State</Typography>,
          field: 'message',
          type: 'string',
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.motion_state}</Typography>;
          },
        },
        {
          title: <Typography>Current Floor</Typography>,
          field: 'message',
          type: 'string',
          render: (rowData) => {
            return <Typography className={classes.cellContent}>{rowData.current_floor}</Typography>;
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
              <Typography className={classes.cellContent} data-testid={'lift-table-date'}>
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
