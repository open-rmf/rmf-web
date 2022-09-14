import { render } from '@testing-library/react';
import React from 'react';
import { RobotIssues } from './robot-issues';
import { makeRobot } from './test-utils.spec';

describe('RobotIssues', () => {
  it('information renders correctly', () => {
    const root = render(<RobotIssues robotIssues={makeRobot({ name: 'test_robot' }).issues} />);
    expect(() => root.getByText('object category')).not.toThrow();
    expect(() => root.getByText('first_field')).not.toThrow();
    expect(() => root.getByText('first_value')).not.toThrow();
    expect(() => root.getByText('second_field')).not.toThrow();
    expect(() => root.getByText('second_value')).not.toThrow();
    expect(() => root.getByText('array category')).not.toThrow();
    expect(() => root.getByText('first_item')).not.toThrow();
    expect(() => root.getByText('second_item')).not.toThrow();
    expect(() => root.getByText('string category')).not.toThrow();
    expect(() => root.getByText('string value')).not.toThrow();
  });
});
