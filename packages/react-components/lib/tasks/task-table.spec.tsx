import { render } from '@testing-library/react';
import React from 'react';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import { TaskTable } from './task-table';
import { makeTaskSummaryWithPhases } from './test-data.spec';

describe('TaskTable', () => {
  it('shows all tasks', () => {
    const tasks = [
      makeTaskSummaryWithPhases('task_0', 3, 3),
      makeTaskSummaryWithPhases('task_1', 2, 2),
    ];
    const root = render(<TaskTable tasks={tasks} />);
    root.getByText('task_0');
    root.getByText('task_1');
  });

  it('smoke test for different task states', () => {
    const activeTask = makeTaskSummaryWithPhases('active_task', 3, 3);
    activeTask.state = RmfTaskSummary.STATE_ACTIVE;
    const cancelledTask = makeTaskSummaryWithPhases('cancelled_task', 3, 3);
    cancelledTask.state = RmfTaskSummary.STATE_CANCELED;
    const completedTask = makeTaskSummaryWithPhases('completed_task', 3, 3);
    completedTask.state = RmfTaskSummary.STATE_COMPLETED;
    const failedTask = makeTaskSummaryWithPhases('failed_task', 3, 3);
    failedTask.state = RmfTaskSummary.STATE_FAILED;
    const pendingTask = makeTaskSummaryWithPhases('pending_task', 3, 3);
    pendingTask.state = RmfTaskSummary.STATE_PENDING;
    const queuedTask = makeTaskSummaryWithPhases('queuedTask', 3, 3);
    queuedTask.state = RmfTaskSummary.STATE_QUEUED;

    const tasks = [activeTask, cancelledTask, completedTask, failedTask, pendingTask, queuedTask];
    render(<TaskTable tasks={tasks} />);
  });
});
