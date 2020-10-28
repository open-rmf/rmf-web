import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import TreeView from '@material-ui/lab/TreeView';
import TreeItem from '@material-ui/lab/TreeItem';
import TaskSummaryPanel from '../task-summary-panel';
import fakeTaskSummary from '../../mock/data/task-summary';
import { createMount } from '@material-ui/core/test-utils';

const mount = createMount();

it('Matches snapshot', () => {
  const tasks = Object.values(fakeTaskSummary());
  const root = mount(<TaskSummaryPanel allTasks={tasks} />);
  expect(root).toMatchSnapshot();
  root.unmount();
});

it('Renders tree items', () => {
  const tasks = Object.values(fakeTaskSummary());
  const root = mount(<TaskSummaryPanel allTasks={tasks} />);
  expect(root.find(TreeView).exists()).toBeTruthy();
  expect(root.find(TreeItem).length).toBe(tasks.length);
  root.unmount();
});

it('Show description below the id if the task has an actor', () => {
  const tasks = Object.values(fakeTaskSummary());
  const root = mount(<TaskSummaryPanel allTasks={tasks} />);
  expect(root.html()).toContain('makeStyles-taskActor');
  root.unmount();
});

it('Does not show description above the id if the task has an actor', () => {
  const task = Object.values(fakeTaskSummary())[0];
  task.status = 'Finished';
  const root = mount(<TaskSummaryPanel allTasks={[task]} />);
  expect(root.html().includes('makeStyles-taskActor')).toBeFalsy();
  root.unmount();
});

describe('Components gets correct style on specifics states', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(fakeTaskSummary())[0];
  });

  test('active style is applied ', () => {
    task.state = RomiCore.TaskSummary.STATE_ACTIVE;
    const root = mount(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.find(TreeItem).html()).toContain('makeStyles-active');
    root.unmount();
  });

  test('queue style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_QUEUED;
    const root = mount(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.find(TreeItem).html()).toContain('makeStyles-queued');
    root.unmount();
  });

  test('completed style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_COMPLETED;
    const root = mount(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.find(TreeItem).html()).toContain('makeStyles-completed');
    root.unmount();
  });

  test('failed style is applied', () => {
    task.state = RomiCore.TaskSummary.STATE_FAILED;
    const root = mount(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.find(TreeItem).html()).toContain('makeStyles-failed');
    root.unmount();
  });
});
