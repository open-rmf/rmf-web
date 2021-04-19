import React from 'react';
import { render } from '@testing-library/react';
import { TaskPanel } from '../../lib';
import { makeTask } from '../test-data/tasks';
import userEvent from '@testing-library/user-event';
import * as RmfModels from 'rmf-models';

describe('TaskPanel', () => {
  it('shows detailed information when task is clicked', () => {
    const task = makeTask('test_task', 3, 3);
    task.task_profile.description.task_type.type = RmfModels.TaskType.TYPE_CLEAN;
    task.task_profile.description.clean.start_waypoint = 'test_waypoint';
    const root = render(<TaskPanel tasks={[task]} />);
    userEvent.click(root.getByText('test_task'));
    root.getByText('test_waypoint');
  });
});
