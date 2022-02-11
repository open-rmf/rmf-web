import React from 'react';
import { render } from '../../tests/test-utils';
import { TaskLogs } from '../task-logs';
import { makeTaskEventLog, makeTaskEventState } from './make-tasks';

// react-leaflet doesn't work well in jsdom.
jest.mock('./../../schedule-visualizer', () => () => null);

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();
  const taskLog = makeTaskEventLog('0');
  const taskState = makeTaskEventState('0');

  const root = render(<TaskLogs taskLog={taskLog} taskState={taskState} />);
  root.unmount();
  (URL.createObjectURL as jest.Mock).mockReset();
});

export {};
