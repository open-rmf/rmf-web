import { render } from '@testing-library/react';
import React from 'react';
import { TaskTimeline } from './task-timeline';
import { makeTaskState } from './test-data.spec';

describe('Task Timeline', () => {
  it('shows the time for each phase', () => {
    const task = makeTaskState('task_0');
    Object.values(task.phases).forEach((p, idx) => {
      p.unix_millis_start_time = 1000 * idx;
    });
    const root = render(<TaskTimeline taskState={task} />);
    Object.values(task.phases).forEach((p) => {
      expect(() =>
        root.getByText((_, node) => {
          if (!node) {
            return false;
          }
          const hasText = (node) =>
            node.textContent === new Date(p.unix_millis_start_time).toLocaleTimeString();
          const nodeHasText = hasText(node);
          const childrenDontHaveText = Array.from(node.children).every((child) => !hasText(child));
          return nodeHasText && childrenDontHaveText;
        }),
      ).not.toThrow();
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
