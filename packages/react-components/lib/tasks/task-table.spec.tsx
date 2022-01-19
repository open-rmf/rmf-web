import { render } from '@testing-library/react';
import React from 'react';
import { TaskTable } from './task-table';
import { makeTaskState } from './test-data.spec';

describe('TaskTable', () => {
  it('shows all tasks', () => {
    const tasks = [makeTaskState('task_0'), makeTaskState('task_1')];
    const root = render(<TaskTable tasks={tasks} />);
    root.getByText('task_0');
    root.getByText('task_1');
  });
});
