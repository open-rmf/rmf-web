import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { Status2 as RobotStatus } from 'api-client';
import React from 'react';
import { RobotTable, RobotTableData } from './robot-table';
import { makeRobot } from './test-utils.spec';

const allStatuses = Object.values(RobotStatus) as RobotStatus[];

describe('RobotTable', () => {
  it('shows all robots', () => {
    const robots = [makeRobot({ name: 'test_robot1' }), makeRobot({ name: 'test_robot2' })];
    const tableData: RobotTableData[] = robots.map((robot) => ({
      fleet: 'test_fleet',
      name: robot.name,
    }));
    const root = render(<RobotTable robots={tableData} />);
    expect(root.getByText('test_robot1')).toBeTruthy();
    expect(root.getByText('test_robot2')).toBeTruthy();
  });

  it('smoke test for different robot status', () => {
    const robots = allStatuses.map((status) => makeRobot({ name: `${status}_robot`, status }));
    render(
      <RobotTable
        robots={robots.map((robot) => ({
          fleet: 'test_fleet',
          name: robot.name,
          status: robot.status,
        }))}
      />,
    );
  });

  it('onRobotClick is called when row is clicked', () => {
    const onRobotClick = jasmine.createSpy();
    const root = render(
      <RobotTable
        robots={[{ fleet: 'test_fleet', name: 'test_robot' }]}
        onRobotClick={onRobotClick}
      />,
    );
    const robot = root.getByText('test_robot');
    userEvent.click(robot);
    expect(onRobotClick).toHaveBeenCalled();
  });

  it('finish time is shown when it is available', () => {
    const root = render(
      <RobotTable
        robots={[
          { fleet: 'test_fleet', name: 'test_robot', estFinishTime: 1000, lastUpdateTime: 900 },
        ]}
      />,
    );
    expect(() => root.getByText(new Date(1000).toLocaleString())).not.toThrow();
    expect(() => root.getByText(new Date(900).toLocaleString())).not.toThrow();
  });
});
