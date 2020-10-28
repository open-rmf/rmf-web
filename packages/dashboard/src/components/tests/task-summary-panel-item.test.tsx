import { createMount } from '@material-ui/core/test-utils';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { shallow } from 'enzyme';
import React from 'react';
import fakeTaskSummary from '../../mock/data/task-summary';
import { TaskSummaryPanelItem } from '../task-summary-panel-item';

const mount = createMount();

it('Matches snapshot', () => {
  const task = Object.values(fakeTaskSummary())[0];
  const root = shallow(<TaskSummaryPanelItem task={task} />);
  expect(root).toMatchSnapshot();
  root.unmount();
});

describe('Components gets the correct label on specifics states', () => {
  let task: RomiCore.TaskSummary;
  beforeEach(() => {
    task = Object.values(fakeTaskSummary())[0];
  });

  test('Shows ACTIVE label', () => {
    task.state = RomiCore.TaskSummary.STATE_ACTIVE;
    const root = mount(<TaskSummaryPanelItem task={task} />);
    expect(root.html()).toContain('ACTIVE');
    root.unmount();
  });

  test('Shows QUEUE label', () => {
    task.state = RomiCore.TaskSummary.STATE_QUEUED;
    const root = mount(<TaskSummaryPanelItem task={task} />);
    expect(root.html()).toContain('QUEUE');
    root.unmount();
  });

  test('Shows COMPLETED label', () => {
    task.state = RomiCore.TaskSummary.STATE_COMPLETED;
    const root = mount(<TaskSummaryPanelItem task={task} />);
    expect(root.html()).toContain('COMPLETED');
    root.unmount();
  });

  test('Shows FAILED label', () => {
    task.state = RomiCore.TaskSummary.STATE_FAILED;
    const root = mount(<TaskSummaryPanelItem task={task} />);
    expect(root.html()).toContain('FAILED');
    root.unmount();
  });
});
