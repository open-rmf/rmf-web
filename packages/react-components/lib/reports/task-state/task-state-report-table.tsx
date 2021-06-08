import React from 'react';
import MaterialTable from 'material-table';
import { Typography } from '@material-ui/core';
import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';

export type TaskStateRowsType = {
  created: string; //date
  payload: string | unknown;
  fleet_name: string;
  task_id: string;
  task_state: string;
  status: string;
  submission_time: string;
  start_time: string;
  end_time: string;
  robot_task_id: string;
}[];

export interface TaskStateReportTable extends DefaultLogTableProps {
  rows: TaskStateRowsType | [];
}

export const TaskStateReportTable = (props: TaskStateReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <MaterialTable
      title="Task State"
      icons={materialTableIcons}
      columns={[
        {
          title: <Typography>Fleet</Typography>,
          field: 'fleet_name',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.fleet_name}</Typography>;
          },
        },
        {
          title: <Typography>Task ID</Typography>,
          field: 'task_id',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.task_id}</Typography>;
          },
        },
        {
          title: <Typography>Task State</Typography>,
          field: 'task_state',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.task_state}</Typography>;
          },
        },
        {
          title: <Typography>Status</Typography>,
          field: 'status',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.status}</Typography>;
          },
        },
        {
          title: <Typography>Submission Time</Typography>,
          field: 'submission_time',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.submission_time}</Typography>;
          },
        },
        {
          title: <Typography>Assigned Robot</Typography>,
          field: 'robot_task_id',
          type: 'string',
          render: (rowData) => {
            return <Typography>{rowData.robot_task_id}</Typography>;
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
              <Typography data-testid={'task-table-date'}>
                {format(new Date(rowData.created), 'MMM dd yyyy hh:mm aaa')}
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
