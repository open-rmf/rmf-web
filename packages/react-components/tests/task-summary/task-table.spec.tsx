import React from 'react';
import { render, waitFor } from '@testing-library/react';
import { TaskTable } from '../../lib';
import { makeTask } from '../test-data/tasks';
import userEvent from '@testing-library/user-event';
import * as RmfModels from 'rmf-models';

describe('TaskTable', () => {
  it('shows all tasks', () => {
    const tasks = [makeTask('task_0', 3, 3), makeTask('task_1', 2, 2)];
    const root = render(<TaskTable tasks={tasks} />);
    root.getByText('task_0');
    root.getByText('task_1');
  });

  it('clicking on create task button opens the create task form', () => {
    const root = render(<TaskTable tasks={[]} />);
    userEvent.click(root.getByLabelText('Create Task'));
    root.getByText('Create Task');
  });

  it('smoke test for different task states', () => {
    const activeTask = makeTask('active_task', 3, 3);
    activeTask.state = RmfModels.TaskSummary.STATE_ACTIVE;
    const cancelledTask = makeTask('cancelled_task', 3, 3);
    cancelledTask.state = RmfModels.TaskSummary.STATE_CANCELED;
    const completedTask = makeTask('completed_task', 3, 3);
    completedTask.state = RmfModels.TaskSummary.STATE_COMPLETED;
    const failedTask = makeTask('failed_task', 3, 3);
    failedTask.state = RmfModels.TaskSummary.STATE_FAILED;
    const pendingTask = makeTask('pending_task', 3, 3);
    pendingTask.state = RmfModels.TaskSummary.STATE_PENDING;
    const queuedTask = makeTask('queuedTask', 3, 3);
    queuedTask.state = RmfModels.TaskSummary.STATE_QUEUED;

    const tasks = [activeTask, cancelledTask, completedTask, failedTask, pendingTask, queuedTask];
    render(<TaskTable tasks={tasks} />);
  });

  it('pagination is shown when pagination option is provided', () => {
    const spy = jasmine.createSpy();
    const root = render(
      <TaskTable
        tasks={[makeTask('test', 1, 1)]}
        paginationOptions={{
          count: 1,
          page: 0,
          rowsPerPage: 10,
          rowsPerPageOptions: [10],
          onChangePage: spy,
        }}
      />,
    );
    root.getByText('1-1 of 1');
  });

  it('success snackbar is shown when successfully created a task', async () => {
    const spy = jasmine.createSpy().and.resolveTo(undefined);
    const root = render(<TaskTable tasks={[]} submitTask={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    userEvent.click(root.getByLabelText('Submit'));
    await waitFor(() => root.getByText('Successfully created task'));
  });

  it('failure snackbar is shown when failed to created a task', async () => {
    const spy = jasmine.createSpy().and.rejectWith(new Error('error!!'));
    const root = render(<TaskTable tasks={[]} submitTask={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    userEvent.click(root.getByLabelText('Submit'));
    await waitFor(() => root.getByText('Failed to create task', { exact: false }));
  });
});
