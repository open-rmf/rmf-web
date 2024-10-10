import { it, Mock, vi } from 'vitest';

import { render } from '../../utils/test-utils.test';
import { makeTaskLog, makeTaskState } from './make-tasks.test';
import { TaskLogs } from './task-logs';

it('renders without crashing', async () => {
  URL.createObjectURL = vi.fn();
  const taskLog = makeTaskLog('0');
  const taskState = makeTaskState('0');

  const root = render(<TaskLogs taskLog={taskLog} taskState={taskState} />);
  root.unmount();
  (URL.createObjectURL as Mock).mockReset();
});

export {};
