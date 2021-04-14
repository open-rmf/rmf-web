import { Meta, Story } from '@storybook/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { PaginationOptions, TaskTable, TaskTableProps } from '../../lib';
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
  const [page, setPage] = React.useState(0);
  const paginationOptions: PaginationOptions = {
    count: args.tasks.length,
    rowsPerPage: 10,
    rowsPerPageOptions: [10],
    page,
    onChangePage: (_ev, newPage) => setPage(newPage),
  };
  return (
    <div style={{ height: '95vh' }}>
      <TaskTable
        tasks={args.tasks.slice(page * 10, (page + 1) * 10)}
        paginationOptions={paginationOptions}
        submitTask={async () => new Promise((res) => setTimeout(res, 1000))}
      />
    </div>
  );
};

Table.args = {
  tasks,
};
