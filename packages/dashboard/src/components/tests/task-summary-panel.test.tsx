import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import TreeView from '@material-ui/lab/TreeView';
import TreeItem from '@material-ui/lab/TreeItem';
import TaskSummaryPanel, { getActorFromStatus, TaskSummaryPanelInfo } from '../task-summary-panel';
import fakeTaskSummary from '../../mock/data/task-summary';
import { createMount } from '@material-ui/core/test-utils';
import { shallow } from 'enzyme';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
const mount = createMount();

describe('Renders correctly', () => {
  test('Renders tree items', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = render(<TaskSummaryPanel allTasks={tasks} />);
    tasks.forEach((task) => {
      expect(root.getByText(task.task_id).textContent).toBe(task.task_id);
    });
  });

  test('Show description below the id if the task has an actor', () => {
    const task = Object.values(fakeTaskSummary())[0];
    const root = render(<TaskSummaryPanel allTasks={[task]} />);
    const actor = getActorFromStatus(task.status);
    if (!actor) throw new Error('An actor is required to run this test');
    const classes = root.getByText(actor[0]).className;
    expect(classes).toContain('makeStyles-taskActor');
  });

  test('Does not show description below the id if the task has an actor', () => {
    const task = Object.values(fakeTaskSummary())[0];
    task.status = 'Finished';
    const root = render(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.container.querySelector('[id=task-actor]')).toBeFalsy();
  });
});

describe('Button Bar working correctly', () => {
  test('should set disabled to true on buttons when empty tasks is provided', () => {
    const root = render(<TaskSummaryPanel allTasks={[]} />);
    expect(
      root.container.querySelector('button#clear-button')?.hasAttribute('disabled'),
    ).toBeTruthy();
    expect(
      root.container.querySelector('button#reset-button')?.hasAttribute('disabled'),
    ).toBeTruthy();
    expect(
      root.container.querySelector('button#restore-button')?.hasAttribute('disabled'),
    ).toBeTruthy();
  });

  test('should set disabled to false on buttons when tasks are provided', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = render(<TaskSummaryPanel allTasks={tasks} />);
    expect(
      root.container.querySelector('button#clear-button')?.hasAttribute('disabled'),
    ).toBeFalsy();
    expect(
      root.container.querySelector('button#reset-button')?.hasAttribute('disabled'),
    ).toBeFalsy();
    expect(
      root.container.querySelector('button#restore-button')?.hasAttribute('disabled'),
    ).toBeFalsy();
  });

  test('should empty all current tasks when clear button is clicked', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = render(<TaskSummaryPanel allTasks={tasks} />);
    userEvent.click(root.getByText('Clear'));
    expect(root.container.querySelector('li')).toBeFalsy();
  });

  test('should render all tasks when restore button is clicked', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = render(<TaskSummaryPanel allTasks={tasks} />);
    // clear all tasks first to ensure empty array
    userEvent.click(root.getByText('Clear'));
    expect(root.container.querySelector('li')).toBeFalsy();
    userEvent.click(root.getByText('Restore'));
    tasks.forEach((task) => {
      expect(root.getByText(task.task_id).textContent).toBe(task.task_id);
    });
  });
});

describe('Components gets the correct style on specifics states', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(fakeTaskSummary())[0];
  });

  test('active style is applied ', () => {
    task.state = RomiCore.TaskSummary.STATE_ACTIVE;
    const root = render(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-active');
  });

  test('queue style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_QUEUED;
    const root = render(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-queued');
  });

  test('completed style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_COMPLETED;
    const root = render(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-completed');
  });

  test('failed style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_FAILED;
    const root = render(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.getByText(task.task_id).parentElement?.className).toContain('makeStyles-failed');
  });
});

describe('Components gets the correct label on specifics states', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(fakeTaskSummary())[0];
  });

  test('Shows ACTIVE label', () => {
    task.state = RomiCore.TaskSummary.STATE_ACTIVE;
    const root = render(<TaskSummaryPanelInfo task={task} />);
    expect(root.getByText('ACTIVE')).toBeTruthy();
  });

  test('Shows QUEUE label', () => {
    task.state = RomiCore.TaskSummary.STATE_QUEUED;
    const root = render(<TaskSummaryPanelInfo task={task} />);
    expect(root.getByText('QUEUED')).toBeTruthy();
  });

  test('Shows COMPLETED label', () => {
    task.state = RomiCore.TaskSummary.STATE_COMPLETED;
    const root = render(<TaskSummaryPanelInfo task={task} />);
    expect(root.getByText('COMPLETED')).toBeTruthy();
  });

  test('Shows FAILED label', () => {
    task.state = RomiCore.TaskSummary.STATE_FAILED;
    const root = render(<TaskSummaryPanelInfo task={task} />);
    expect(root.getByText('FAILED')).toBeTruthy();
  });
});

test('Get name of the actor from status', () => {
  const rawStatus = 'Finding a plan for [tinyRobot/tinyRobot1] to go to [23] | Remaining phases: 6';
  const actor = getActorFromStatus(rawStatus);
  expect(actor).toEqual(['[tinyRobot/tinyRobot1]']);
});
