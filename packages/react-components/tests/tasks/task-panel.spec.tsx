import { render, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskPanel, TaskPanelProps } from '../../lib';
import { makeTask } from '../test-data/tasks';

function makeFetchTasks(tasks: RmfModels.TaskSummary[]): TaskPanelProps['fetchTasks'] {
  return async (limit, offset) => {
    return {
      tasks: tasks.slice(offset, offset + limit),
      totalCount: tasks.length,
    };
  };
}

describe('TaskPanel', () => {
  it('shows detailed information when task is clicked', async () => {
    const task = makeTask('test_task', 3, 3);
    task.task_profile.description.task_type.type = RmfModels.TaskType.TYPE_CLEAN;
    task.task_profile.description.clean.start_waypoint = 'test_waypoint';
    const root = render(<TaskPanel fetchTasks={makeFetchTasks([task])} />);
    userEvent.click(await root.findByText('test_task'));
    root.getByText('test_waypoint');
  });

  it('clicking on create task button opens the create task form', () => {
    const root = render(<TaskPanel fetchTasks={makeFetchTasks([])} />);
    userEvent.click(root.getByLabelText('Create Task'));
    root.getByText('Create Task');
  });

  it('success snackbar is shown when successfully created a task', async () => {
    const spy = jasmine.createSpy().and.resolveTo(undefined);
    const root = render(<TaskPanel fetchTasks={makeFetchTasks([])} submitTask={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    userEvent.click(root.getByLabelText('Submit'));
    await waitFor(() => root.getByText('Successfully created task'));
  });

  it('failure snackbar is shown when failed to created a task', async () => {
    const spy = jasmine.createSpy().and.rejectWith(new Error('error!!'));
    const root = render(<TaskPanel fetchTasks={makeFetchTasks([])} submitTask={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    userEvent.click(root.getByLabelText('Submit'));
    await waitFor(() => root.getByText('Failed to create task', { exact: false }));
  });
});
