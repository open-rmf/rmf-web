import { it, Mock, vi } from 'vitest';

import { render } from '../../tests/test-utils';
import { TaskLogs } from '../task-logs';
import { makeTaskLog, makeTaskState } from './make-tasks';

it('renders without crashing', async () => {
  URL.createObjectURL = vi.fn();
  const taskLog = makeTaskLog('0');
  const taskState = makeTaskState('0');

  const root = render(<TaskLogs taskLog={taskLog} taskState={taskState} />);
  root.unmount();
  (URL.createObjectURL as Mock).mockReset();
});

export {};
