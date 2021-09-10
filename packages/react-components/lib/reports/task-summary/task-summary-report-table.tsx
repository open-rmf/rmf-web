import React from 'react';
import { DataGrid } from '@mui/x-data-grid';
import { Typography } from '@material-ui/core';
// import { materialTableIcons } from '../../material-table-icons';
import { DefaultLogTableProps } from '../default-report-interface';
import { format } from 'date-fns';
import { returnTaskDetails } from './utils';
import { rosTimeToJs } from '../../utils';
import * as RmfModels from 'rmf-models';

export type TaskSummaryRowsType = {
  created: string; //date
  fleet: { id: number; name: string };
  robot: { id: number; name: string; model?: string };
  task_id: string;
  task_profile: RmfModels.TaskProfile;
  state: string;
  status: string;
  submission_time: RmfModels.Time;
  start_time: RmfModels.Time;
  end_time: RmfModels.Time;
}[];

export interface TaskSummaryReportTable extends DefaultLogTableProps {
  rows: TaskSummaryRowsType | [];
}

export const TaskSummaryReportTable = (props: TaskSummaryReportTable): React.ReactElement => {
  const { rows, tableSize, addMoreRows } = props;

  return (
    <DataGrid
      // title="Task Summary"
      // icons={materialTableIcons}
      columns={[
        {
          headerName: 'Task ID',
          field: 'task_id',
          type: 'string',
          valueFormatter: (rowData) => {
            return <Typography>{rowData.row.task_id}</Typography>;
          },
        },
        {
          headerName: 'Fleet',
          field: 'fleet_name',
          type: 'string',
          valueFormatter: (rowData) => {
            return <Typography>{rowData.row.fleet.name}</Typography>;
          },
        },
        {
          headerName: 'Robot',
          field: 'robot_name',
          type: 'string',
          valueFormatter: (rowData) => {
            return <Typography>{rowData.row.robot.name}</Typography>;
          },
        },
        {
          headerName: 'Task Description',
          field: 'description',
          type: 'string',
          valueFormatter: (rowData) => {
            const taskTypeDetails = returnTaskDetails(
              rowData.row.task_id,
              rowData.row.task_profile.description,
            );
            return taskTypeDetails;
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
          headerName: 'Time',
          field: 'time_information',
          type: 'string',
          valueFormatter: (rowData) => {
            const submissionTime = rosTimeToJs(
              rowData.row.task_profile.submission_time,
            ).toLocaleTimeString();
            const startTime = rosTimeToJs(rowData.row.start_time).toLocaleTimeString();
            const endTime = rosTimeToJs(rowData.row.end_time).toLocaleTimeString();
            return (
              <>
                <Typography>Submitted: {submissionTime}</Typography>
                <Typography>Start: {startTime}</Typography>
                <Typography>End: {endTime}</Typography>
              </>
            );
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
              <Typography data-testid={'task-table-date'}>
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
  );
};
