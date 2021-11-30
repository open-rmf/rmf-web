import { Paper, TableContainer, TablePagination } from '@mui/material';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import { TaskTable, TaskTableProps } from './task-table';
import { makeTaskSummaryWithPhases } from './test-data.spec';

const failedTask = makeTaskSummaryWithPhases('failed_task', 3, 3);
failedTask.state = RmfTaskSummary.STATE_FAILED;

const pendingTask = makeTaskSummaryWithPhases('pending_task', 3, 0);
pendingTask.state = RmfTaskSummary.STATE_PENDING;

const queuedTask = makeTaskSummaryWithPhases('pending_task', 3, 0);
queuedTask.state = RmfTaskSummary.STATE_QUEUED;

const completedtasks = Array.from(Array(100)).map((_, idx) => {
  const task = makeTaskSummaryWithPhases(`completed_task_${idx}`, 3, 3);
  task.state = RmfTaskSummary.STATE_COMPLETED;
  return task;
});

const tasks = [
  makeTaskSummaryWithPhases('active_task', 3, 3),
  makeTaskSummaryWithPhases('active_task_2', 4, 3),
  failedTask,
  pendingTask,
  queuedTask,
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
  const [page, setPage] = React.useState(0);
  const count = page >= args.tasks.length / 10 - 1 ? args.tasks.length : -1;

  return (
    <Paper>
      <TableContainer>
        <TaskTable {...args} tasks={args.tasks.slice(page * 10, (page + 1) * 10)} />
      </TableContainer>
      <TablePagination
        component="div"
        count={count}
        rowsPerPage={10}
        rowsPerPageOptions={[10]}
        page={page}
        onPageChange={(_ev, newPage) => setPage(newPage)}
      />
    </Paper>
  );
};

Table.args = {
  tasks,
};
