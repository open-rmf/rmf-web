import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import TreeView from '@material-ui/lab/TreeView';
import TreeItem from '@material-ui/lab/TreeItem';
import TaskSummaryPanel, { getActorFromStatus } from '../task-summary-panel';
import fakeTaskSummary from '../../mock/data/task-summary';
import { createMount } from '@material-ui/core/test-utils';
import { shallow } from 'enzyme';

const mount = createMount();

describe('Renders correctly', () => {
  test('Matches snapshot', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = shallow(<TaskSummaryPanel allTasks={tasks} />);
    expect(root).toMatchSnapshot();
    root.unmount();
  });

  test('Renders tree items', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = mount(<TaskSummaryPanel allTasks={tasks} />);
    expect(root.find(TreeView).exists()).toBeTruthy();
    expect(root.find(TreeItem).length).toBe(tasks.length);
    root.unmount();
  });

  test('Show description below the id if the task has an actor', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = mount(<TaskSummaryPanel allTasks={tasks} />);
    expect(root.html()).toContain('makeStyles-taskActor');
    root.unmount();
  });

  test('Does not show description below the id if the task has an actor', () => {
    const task = Object.values(fakeTaskSummary())[0];
    task.status = 'Finished';
    const root = mount(<TaskSummaryPanel allTasks={[task]} />);
    expect(root.html().includes('makeStyles-taskActor')).toBeFalsy();
    root.unmount();
  });
});

describe('Button Bar working correctly', () => {
  test('should set disabled to true on buttons when empty tasks is provided', () => {
    const root = mount(<TaskSummaryPanel allTasks={[]} />);
    expect(root.find('button#clear-button').props().disabled).toEqual(true);
    expect(root.find('button#reset-button').props().disabled).toEqual(true);
    expect(root.find('button#restore-button').props().disabled).toEqual(true);
  });

  test('should set disabled to false on buttons when tasks are provided', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = mount(<TaskSummaryPanel allTasks={tasks} />);
    expect(root.find('button#clear-button').props().disabled).toEqual(false);
    expect(root.find('button#reset-button').props().disabled).toEqual(false);
    expect(root.find('button#restore-button').props().disabled).toEqual(false);
  });

  test('should empty all current tasks when clear button is clicked', () => {
    const tasks = Object.values(fakeTaskSummary());
    const root = mount(<TaskSummaryPanel allTasks={tasks} />);
    root.find('button#clear-button').simulate('click');
    expect(root.find('li').length).toEqual(0);
    root.unmount();
  });

  test('should render all tasks when restore button is clicked', () => {
    const task = Object.values(fakeTaskSummary())[0];
    const root = mount(<TaskSummaryPanel allTasks={[task]} />);
    // clear all tasks first to ensure empty array
    root.find('button#clear-button').simulate('click');
    root.find('button#restore-button').simulate('click');
    expect(root.find('li').length).toEqual(1);
    root.unmount();
  });
});

describe('Components gets the correct style on specifics states', () => {
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

test('Get name of the actor from status', () => {
  const rawStatus = 'Finding a plan for [tinyRobot/tinyRobot1] to go to [23] | Remaining phases: 6';
  const actor = getActorFromStatus(rawStatus);
  expect(actor).toEqual(['[tinyRobot/tinyRobot1]']);
});
