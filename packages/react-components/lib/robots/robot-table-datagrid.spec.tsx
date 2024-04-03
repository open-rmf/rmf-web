import { render, fireEvent } from '@testing-library/react';
import { ApiServerModelsRmfApiRobotStateStatus as RobotStatus } from 'api-client';
import React from 'react';
import { RobotTableData } from './robot-table';
import { makeRobot } from './test-utils.spec';
import { RobotDataGridTable } from './robot-table-datagrid';

const allStatuses = Object.values(RobotStatus) as RobotStatus[];

describe('RobotTable', () => {
  it('shows all robots', () => {
    const robots = [makeRobot({ name: 'test_robot1' }), makeRobot({ name: 'test_robot2' })];
    const tableData: RobotTableData[] = robots.map((robot) => ({
      fleet: 'test_fleet',
      name: robot.name!,
    }));
    const root = render(<RobotDataGridTable robots={tableData} />);
    expect(root.getByText('test_robot1')).toBeTruthy();
    expect(root.getByText('test_robot2')).toBeTruthy();
  });

  it('smoke test for different robot status', () => {
    const robots = allStatuses.map((status) => makeRobot({ name: `${status}_robot`, status }));
    render(
      <RobotDataGridTable
        robots={robots.map((robot) => ({
          fleet: 'test_fleet',
          name: robot.name!,
          status: robot.status,
        }))}
      />,
    );
  });

  it('onRobotClick is called when row is clicked', () => {
    const onRobotClick = jest.fn();
    const root = render(
      <RobotDataGridTable
        robots={[{ fleet: 'test_fleet', name: 'test_robot' }]}
        onRobotClick={onRobotClick}
      />,
    );
    const robot = root.getByText('test_robot');
    fireEvent.click(robot);
    expect(onRobotClick).toHaveBeenCalled();
  });

  it('finish time is shown when it is available', () => {
    const root = render(
      <RobotDataGridTable
        robots={[
          { fleet: 'test_fleet', name: 'test_robot', estFinishTime: 1000, lastUpdateTime: 900 },
        ]}
      />,
    );
    // TODO: use a less convoluted test when
    // https://github.com/testing-library/react-testing-library/issues/1160
    // is resolved.
    expect(() =>
      root.getByText((_, node) => {
        if (!node) {
          return false;
        }
        const hasText = (node: Element) => node.textContent === new Date(1000).toLocaleString();
        const nodeHasText = hasText(node);
        const childrenDontHaveText = Array.from(node.children).every((child) => !hasText(child));
        return nodeHasText && childrenDontHaveText;
      }),
    ).not.toThrow();
    expect(() =>
      root.getByText((_, node) => {
        if (!node) {
          return false;
        }
        const hasText = (node: Element) => node.textContent === new Date(900).toLocaleString();
        const nodeHasText = hasText(node);
        const childrenDontHaveText = Array.from(node.children).every((child) => !hasText(child));
        return nodeHasText && childrenDontHaveText;
      }),
    ).not.toThrow();
  });
});
