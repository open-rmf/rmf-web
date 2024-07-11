import { render } from '@testing-library/react';
import React from 'react';
import { RobotInfo } from './robot-info';

describe('RobotInfo', () => {
  it('information renders correctly', () => {
    const root = render(
      <RobotInfo
        robotName="test_robot"
        assignedTask="test_task"
        battery={0.5}
        estFinishTime={0}
        taskProgress={0.6}
        taskStatus="underway"
      />,
    );
    expect(() => root.getByText('test_robot')).not.toThrow();
    expect(() => root.getByText('test_task')).not.toThrow();
    expect(() => root.getByText('50.00%')).not.toThrow(); // battery
    expect(() => root.getByText('60%')).not.toThrow(); // task progress
    expect(() => root.getByText(/.*underway/)).not.toThrow();
    // TODO: use a less convoluted test when
    // https://github.com/testing-library/react-testing-library/issues/1160
    // is resolved.
    expect(() =>
      root.getByText((_, node) => {
        if (!node) {
          return false;
        }
        const hasText = (node: Element) => node.textContent === new Date(0).toLocaleString();
        const nodeHasText = hasText(node);
        const childrenDontHaveText = Array.from(node.children).every((child) => !hasText(child));
        return nodeHasText && childrenDontHaveText;
      }),
    ).not.toThrow();
  });

  describe('Task status', () => {
    it('shows no task when there is no assigned task and task status', () => {
      const root = render(<RobotInfo robotName="test_robot" />);
      expect(() => root.getByText(/No Task/)).not.toThrow();
    });

    it('shows unknown when there is an assigned task but no status', () => {
      const root = render(
        <RobotInfo
          robotName="test_robot"
          battery={0.5}
          estFinishTime={0}
          assignedTask="test_task"
        />,
      );
      expect(() => root.getByText(/Unknown/)).not.toThrow();
    });
  });

  it('defaults to 0% when no battery is available', () => {
    const root = render(<RobotInfo robotName="test_robot" />);
    expect(() => root.getByText('0%')).not.toThrow();
  });
});
