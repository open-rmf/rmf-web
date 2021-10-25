import { render } from '@testing-library/react';
import React from 'react';
import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import { rosTimeToJs } from '../utils';
import { TaskTimeline } from './task-timeline';
import { makeTaskSummaryWithPhases } from './test-data.spec';

describe('Task Timeline', () => {
  it('shows the time', () => {
    const task = makeTaskSummaryWithPhases('task_0', 2, 1);
    const root = render(<TaskTimeline taskSummary={task} />);
    const expectedTime = rosTimeToJs(task.start_time).toLocaleTimeString();
    expect(root.getAllByText(expectedTime).length).toBeGreaterThan(0);
  });

  it('shows all task phases', () => {
    const task = makeTaskSummaryWithPhases('task_0', 3, 3);
    const root = render(<TaskTimeline taskSummary={task} />);
    expect(root.getByText(/Phase 1 test phase 1/i)).toBeTruthy();
    expect(root.getByText(/Phase 2 test phase 2/i)).toBeTruthy();
    expect(root.getByText(/Phase 3 test phase 3/i)).toBeTruthy();
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
    const timelines = tasks.map((task, index) => {
      render(<TaskTimeline key={index} taskSummary={task} />);
    });
    return timelines;
  });
});
