import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { render } from '@testing-library/react';
import { TaskSummaryAccordion, TaskSummaryAccordionInfo } from '../../lib';
import { getActorFromStatus, sortTasksByState } from '../../lib/task-summary/task-summary-utils';

function getTaskObject(): Record<string, RomiCore.TaskSummary> {
  // Returns a task object with a new memory allocation
  return {
    'af8faee9-84ca-41ea-8bb6-8493cc9f824c': {
      end_time: { sec: 0, nanosec: 0 },
      start_time: { sec: 0, nanosec: 0 },
      state: 1,
      status:
        'Moving [tinyRobot/tinyRobot1]: ( 9.81228 -6.98942 -3.12904) -> ( 6.26403 -3.51569  1.16864) | Remaining phases: 1 | Remaining phases: 6',
      submission_time: { sec: 0, nanosec: 0 },
      task_id: 'af8faee9-84ca-41ea-8bb6-8493cc9f824c',
    },
    'am8faee9-84ca-41ea-8bb6-8493cc9f8249': {
      end_time: { sec: 0, nanosec: 0 },
      start_time: { sec: 0, nanosec: 0 },
      state: 0,
      status:
        'Moving [tinyRobot/tinyRobot2]: ( 9.81228 -6.98942 -3.12904) -> ( 6.26403 -3.51569  1.16864) | Remaining phases: 1 | Remaining phases: 6',
      submission_time: { sec: 0, nanosec: 0 },
      task_id: 'am8faee9-84ca-41ea-8bb6-8493cc9f8249',
    },
  };
}

describe('Renders correctly', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(getTaskObject())[0];
  });

  test('Renders tree items', () => {
    const tasks = Object.values(getTaskObject());
    const root = render(<TaskSummaryAccordion tasks={tasks} />);
    tasks.forEach((task) => {
      expect(root.getByText(task.task_id).textContent).toBe(task.task_id);
    });
  });

  test('Show description below the id if the task has an actor', () => {
    const root = render(<TaskSummaryAccordion tasks={[task]} />);
    const actor = getActorFromStatus(task.status);
    if (!actor) throw new Error('An actor is required to run this test');
    const classes = root.getByText(actor[0]).className;
    expect(classes).toContain('makeStyles-taskActor');
  });

  test('Does not show description below the id if the task has no actor', () => {
    task.status = 'Finished';
    const root = render(<TaskSummaryAccordion tasks={[task]} />);
    expect(root.container.querySelector('[id=task-actor]')).toBeFalsy();
  });
});

describe('Components gets the correct style on specifics states', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(getTaskObject())[0];
  });

  test('active style is applied ', () => {
    task.state = RomiCore.TaskSummary.STATE_ACTIVE;
    const root = render(<TaskSummaryAccordion tasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-active');
  });

  test('queue style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_QUEUED;
    const root = render(<TaskSummaryAccordion tasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-queued');
  });

  test('completed style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_COMPLETED;
    const root = render(<TaskSummaryAccordion tasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-completed');
  });

  test('failed style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_FAILED;
    const root = render(<TaskSummaryAccordion tasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-failed');
  });
});

describe('Components gets the correct label on specifics states', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(getTaskObject())[0];
  });

  test('Shows ACTIVE label', () => {
    task.state = RomiCore.TaskSummary.STATE_ACTIVE;
    const root = render(<TaskSummaryAccordionInfo task={task} />);
    expect(root.getByText('ACTIVE')).toBeTruthy();
  });

  test('Shows QUEUE label', () => {
    task.state = RomiCore.TaskSummary.STATE_QUEUED;
    const root = render(<TaskSummaryAccordionInfo task={task} />);
    expect(root.getByText('QUEUED')).toBeTruthy();
  });

  test('Shows COMPLETED label', () => {
    task.state = RomiCore.TaskSummary.STATE_COMPLETED;
    const root = render(<TaskSummaryAccordionInfo task={task} />);
    expect(root.getByText('COMPLETED')).toBeTruthy();
  });

  test('Shows FAILED label', () => {
    task.state = RomiCore.TaskSummary.STATE_FAILED;
    const root = render(<TaskSummaryAccordionInfo task={task} />);
    expect(root.getByText('FAILED')).toBeTruthy();
  });
});

test('Get name of the actor from status', () => {
  const rawStatus = 'Finding a plan for [tinyRobot/tinyRobot1] to go to [23] | Remaining phases: 6';
  const actor = getActorFromStatus(rawStatus);
  expect(actor).toEqual(['[tinyRobot/tinyRobot1]']);
});

test('Sorts task array correctly', () => {
  const tasks = sortTasksByState(getTaskObject());
  expect(tasks[0].state).toBe(RomiCore.TaskSummary.STATE_ACTIVE);
  expect(tasks[1].state).toBe(RomiCore.TaskSummary.STATE_QUEUED);
});
