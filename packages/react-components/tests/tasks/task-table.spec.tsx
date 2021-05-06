import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskTable } from '../../lib';
import { makeTask } from '../test-data/tasks';

describe('TaskTable', () => {
  it('shows all tasks', () => {
    const tasks = [makeTask('task_0', 3, 3), makeTask('task_1', 2, 2)];
    const root = render(<TaskTable tasks={tasks} />);
    root.getByText('task_0');
    root.getByText('task_1');
  });

  it('clicking on create task button trigger onCreateTaskClick', () => {
    const spy = jasmine.createSpy();
    const root = render(<TaskTable tasks={[]} onCreateTaskClick={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  it('clicking on refresh button triggers onRefreshClick', () => {
    const spy = jasmine.createSpy();
    const root = render(<TaskTable tasks={[]} onRefreshClick={spy} />);
    userEvent.click(root.getByLabelText('Refresh'));
    expect(spy).toHaveBeenCalledTimes(1);
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
});
