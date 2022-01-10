import React from 'react';
import { DataGrid, GridRenderCellParams } from '@mui/x-data-grid';
import type { Time } from 'api-client';
import { format } from 'date-fns';
import { Typography } from '@mui/material';
import { rosTimeToJs } from '../../utils';
import { DefaultLogTableProps } from '../default-report-interface';
import { returnTaskDetails } from './utils';

export type TaskSummaryRowsType = {
  created: string; //date
  fleet: { id: number; name: string };
  robot: { id: number; name: string; model?: string };
  task_id: string;
  task_profile: any;
  state: string;
  status: string;
  submission_time: Time;
  start_time: Time;
  end_time: Time;
}[];

export interface TaskSummaryReportTable extends DefaultLogTableProps {
  rows: TaskSummaryRowsType | [];
}

export const TaskSummaryReportTable = (props: TaskSummaryReportTable): React.ReactElement => {
  const { rows, addMoreRows } = props;

  return (
    <div style={{ height: '100%', width: '100%' }}>
      <DataGrid
        autoHeight={true}
        getRowId={(r) => r.task_id}
        columns={[
          {
            headerName: 'Task ID',
            field: 'task_id',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.task_id}</Typography>;
            },
          },
          {
            headerName: 'Fleet',
            field: 'fleet_name',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.fleet.name}</Typography>;
            },
          },
          {
            headerName: 'Robot',
            field: 'robot_name',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.robot.name}</Typography>;
            },
          },
          {
            headerName: 'Task Description',
            field: 'description',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
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
            renderCell: (rowData: GridRenderCellParams) => {
              return <Typography>{rowData.row.state}</Typography>;
            },
          },
          {
            headerName: 'Time',
            field: 'time_information',
            type: 'string',
            renderCell: (rowData: GridRenderCellParams) => {
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
            field: 'created',
            type: 'datetime',
            filterable: false,
            align: 'center',
            renderCell: (rowData: GridRenderCellParams) => {
              return (
                <Typography data-testid={'task-table-date'}>
                  {format(new Date(rowData.value as number), 'MMM dd yyyy hh:mm aaa')}
                </Typography>
              );
            },
          },
        ]}
        rows={rows}
        pageSize={100}
        rowsPerPageOptions={[50, 100]}
        onPageChange={() => {
          if (addMoreRows) {
            addMoreRows();
          }
        }}
        disableColumnMenu={true}
      />
    </div>
  );
};
