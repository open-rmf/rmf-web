import { render, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { TaskPanel } from '../../lib';
import { makeTask } from '../test-data/tasks';

describe('TaskPanel', () => {
  it('shows detailed information when task is clicked', () => {
    const task = makeTask('test_task', 3, 3);
    task.task_profile.description.task_type.type = RmfModels.TaskType.TYPE_CLEAN;
    task.task_profile.description.clean.start_waypoint = 'test_waypoint';
    const root = render(<TaskPanel tasks={[task]} />);
    userEvent.click(root.getByText('test_task'));
    root.getByText('test_waypoint');
  });

  it('clicking on create task button opens the create task form', () => {
    const root = render(<TaskPanel tasks={[]} />);
    userEvent.click(root.getByLabelText('Create Task'));
    root.getByText('Create Task');
  });

  it('success snackbar is shown when successfully created a task', async () => {
    const spy = jasmine.createSpy().and.resolveTo(undefined);
    const root = render(<TaskPanel tasks={[]} submitTask={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    userEvent.click(root.getByLabelText('Submit'));
    await waitFor(() => root.getByText('Successfully created task'));
  });

  it('failure snackbar is shown when failed to created a task', async () => {
    const spy = jasmine.createSpy().and.rejectWith(new Error('error!!'));
    const root = render(<TaskPanel tasks={[]} submitTask={spy} />);
    userEvent.click(root.getByLabelText('Create Task'));
    userEvent.click(root.getByLabelText('Submit'));
    await waitFor(() => root.getByText('Failed to create task', { exact: false }));
  });
});
