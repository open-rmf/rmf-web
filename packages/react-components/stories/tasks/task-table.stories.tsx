import { Snackbar } from '@material-ui/core';
import { Alert, AlertProps } from '@material-ui/lab';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { CreateTaskForm, PaginationOptions, TaskTable, TaskTableProps } from '../../lib';
import { makeTask } from '../../tests/test-data/tasks';

const failedTask = makeTask('failed_task', 3, 3);
failedTask.state = RmfModels.TaskSummary.STATE_FAILED;

const completedtasks = Array.from(Array(100)).map((_, idx) => {
  const task = makeTask(`completed_task_${idx}`, 3, 3);
  task.state = RmfModels.TaskSummary.STATE_COMPLETED;
  return task;
});

const tasks = [
  makeTask('active_task', 3, 3),
  makeTask('active_task_2', 4, 3),
  failedTask,
  ...completedtasks,
];

export default {
  title: 'Tasks/Table',
  component: TaskTable,
  argTypes: {
    paginationOptions: {
      control: {
        disable: true,
      },
    },
    submitTask: {
      control: {
        disable: true,
      },
    },
  },
} as Meta;

export const Table: Story<TaskTableProps> = (args) => {
  const [openCreateTaskForm, setOpenCreateTaskForm] = React.useState(false);
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [snackbarSeverity, setSnackbarSeverity] = React.useState<AlertProps['severity']>('success');
  const [page, setPage] = React.useState(0);
  const paginationOptions: PaginationOptions = {
    count: args.tasks.length,
    rowsPerPage: 10,
    rowsPerPageOptions: [10],
    page,
    onChangePage: (_ev, newPage) => setPage(newPage),
  };
  return (
    <>
      <TaskTable
        {...args}
        style={{ height: '95vh', display: 'flex', flexDirection: 'column' }}
        tasks={args.tasks.slice(page * 10, (page + 1) * 10)}
        paginationOptions={paginationOptions}
        onCreateTaskClick={() => setOpenCreateTaskForm(true)}
      />
      <CreateTaskForm
        cleaningZones={['test_zone_0', 'test_zone_1']}
        loopWaypoints={['test_waypoint_0', 'test_waypoint_1']}
        deliveryWaypoints={['test_waypoint_0', 'test_waypoint_1']}
        dispensers={['test_dispenser_0', 'test_dispenser_1']}
        ingestors={['test_ingestor_0', 'test_ingestor_1']}
        open={openCreateTaskForm}
        onClose={() => setOpenCreateTaskForm(false)}
        submitTasks={async () => new Promise((res) => setTimeout(res, 1000))}
        onCancelClick={() => setOpenCreateTaskForm(false)}
        onSuccess={() => {
          setOpenCreateTaskForm(false);
          setSnackbarSeverity('success');
          setSnackbarMessage('Successfully created task');
          setOpenSnackbar(true);
        }}
        onFail={(e) => {
          setSnackbarSeverity('error');
          setSnackbarMessage(`Failed to create task: ${e.message}`);
          setOpenSnackbar(true);
        }}
      />
      <Snackbar open={openSnackbar} onClose={() => setOpenSnackbar(false)} autoHideDuration={2000}>
        <Alert severity={snackbarSeverity}>{snackbarMessage}</Alert>
      </Snackbar>
    </>
  );
};

Table.args = {
  tasks,
};
