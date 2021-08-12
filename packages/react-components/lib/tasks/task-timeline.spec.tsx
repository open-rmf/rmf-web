import { render } from '@testing-library/react';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskTimeline } from './task-timeline';
import { makeTask } from './test-data.spec';

describe('Task Timeline', () => {
  it('shows the time', () => {
    const task = makeTask('task_0', 3, 3);
    const root = render(<TaskTimeline taskSummary={task} />);
    expect(root.getAllByText('7:30:00AM').length).toBe(2);
  });

  it('shows all task phases', () => {
    const task = makeTask('task_0', 3, 3);
    const root = render(<TaskTimeline taskSummary={task} />);
    expect(root.getByText('Phase 1 test phase 1')).toBeTruthy();
    expect(root.getByText('Phase 2 test phase 2')).toBeTruthy();
    expect(root.getByText('Phase 3 test phase 3')).toBeTruthy();
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
    const timelines = tasks.map((task, index) => {
      render(<TaskTimeline key={index} taskSummary={task} />);
    });
    return timelines;
  });
});
