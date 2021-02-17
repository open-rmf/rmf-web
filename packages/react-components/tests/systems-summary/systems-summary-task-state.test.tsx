import React from 'react';
import { SystemSummaryTaskState } from '../../lib';
import { screen, render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { tasks } from './test.utils';

test('expect all different task states to be calculated and rendered', () => {
  render(<SystemSummaryTaskState tasks={tasks} onClick={jest.fn()} />);
  // 1 task for each state, total 4 states
  expect(screen.queryAllByText('1').length).toEqual(4);
});

test('should call onClick when panel is clicked', () => {
  const onClick = jest.fn();
  render(<SystemSummaryTaskState tasks={tasks} onClick={onClick} />);
  userEvent.click(screen.getByLabelText('panel'));
  expect(onClick).toBeCalled();
});
