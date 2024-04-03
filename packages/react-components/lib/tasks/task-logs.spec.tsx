import { render } from '@testing-library/react';
import React from 'react';
import { TaskLogs } from './task-logs';
import { makeTaskLog } from './test-data.spec';

describe('TaskLogs', () => {
  it('shows all event logs', () => {
    const logs = makeTaskLog('task');
    const root = render(
      <TaskLogs
        taskLog={logs}
        eventName={(eventId) => `Event ${eventId}`}
        eventStatus={() => 'completed'}
      />,
    );
    Object.values(logs.phases!).forEach((p) => {
      Object.values(p.events!).forEach((e) => {
        e.forEach((l) => {
          expect(root.getAllByText(l.text).length).toBeGreaterThan(0);
        });
      });
    });
  });

  it('placeholder is shown when an event has no logs', () => {
    const root = render(
      <TaskLogs
        taskLog={{
          task_id: 'test_task',
          phases: {
            '1': {
              log: [{ seq: 0, text: 'phase log', tier: 'info', unix_millis_time: 0 }],
              events: { '1': [] },
            },
          },
        }}
        eventName={() => 'test_event'}
        eventStatus={() => 'completed'}
      />,
    );
    expect(() => root.getByText('No Logs')).not.toThrow();
  });

  it('placeholder is shown when there are no events', () => {
    const root = render(
      <TaskLogs
        taskLog={{
          task_id: 'test_task',
          phases: {
            '1': {
              log: [{ seq: 0, text: 'phase log', tier: 'info', unix_millis_time: 0 }],
            },
          },
        }}
        eventName={() => 'test_event'}
        eventStatus={() => 'completed'}
      />,
    );
    expect(() => root.getByText('No Event Logs')).not.toThrow();
  });

  it('placeholder is shown where there are no phases', () => {
    const root = render(
      <TaskLogs
        taskLog={{
          task_id: 'test_task',
        }}
        eventName={() => 'test_event'}
        eventStatus={() => 'completed'}
      />,
    );
    expect(() => root.getByText('No logs to be shown')).not.toThrow();
  });
});
