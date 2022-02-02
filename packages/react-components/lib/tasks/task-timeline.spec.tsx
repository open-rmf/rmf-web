import { render } from '@testing-library/react';
import React from 'react';
import { TaskTimeline } from './task-timeline';
import { makeTaskState } from './test-data.spec';

describe('Task Timeline', () => {
  // FIXME: sample phases has no start time
  xit('shows the time for each phase', () => {
    const task = makeTaskState('task_0');
    const root = render(<TaskTimeline taskState={task} />);
    Object.values(task.phases).forEach((p) => {
      expect(p.unix_millis_start_time).toBeTruthy();
      const expectedTime = new Date(p.unix_millis_start_time).toLocaleTimeString();
      expect(() => root.getByText(expectedTime)).not.toThrow();
    });
  });

  it('shows all task events', () => {
    const task = makeTaskState('task_0');
    const root = render(<TaskTimeline taskState={task} />);
    expect(task.phases).toBeTruthy();
    Object.values(task.phases).forEach((p) => {
      if (!p.events) return;
      Object.values(p.events).forEach((e) => {
        expect(() => root.getByText(e.name)).toBeTruthy();
      });
    });
  });
});
