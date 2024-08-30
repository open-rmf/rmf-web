import { render } from '@testing-library/react';
import { describe, it } from 'vitest';

import { TaskInfo } from './task-info';
import { makeTaskState } from './test-data.test';

describe('TaskInfo', () => {
  it('smoke test', () => {
    const task = makeTaskState('task');
    render(
      <>
        <TaskInfo task={task} />
      </>,
    );
  });
});
