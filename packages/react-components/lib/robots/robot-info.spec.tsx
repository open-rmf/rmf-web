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
    expect(() => root.getByText('50%')).not.toThrow(); // task progress
    expect(() => root.getByText('60%')).not.toThrow(); // battery
    expect(() => root.getByText(/.*underway/)).not.toThrow();
    expect(() => root.getByText(new Date(0).toLocaleString())).not.toThrow();
  });
});
