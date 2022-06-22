import React from 'react';
import { render } from '../../tests/test-utils';
import { TaskLogs } from '../task-logs';
import { makeTaskLog, makeTaskState } from './make-tasks';

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();
  const taskLog = makeTaskLog('0');
  const taskState = makeTaskState('0');

  const root = render(<TaskLogs taskLog={taskLog} taskState={taskState} />);
  root.unmount();
  (URL.createObjectURL as jest.Mock).mockReset();
});

export {};
