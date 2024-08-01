import { render } from '@testing-library/react';

import { TaskInfo } from './task-info';
import { makeTaskState } from './test-data.spec';

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
