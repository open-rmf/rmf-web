import { render } from '@testing-library/react';
import { Status2 as RobotStatus } from 'api-client';
import React from 'react';
import { RobotTable, RobotTableData } from './robot-table';
import { makeRobot } from './test-utils.spec';

const allStatuses = Object.values(RobotStatus) as RobotStatus[];

describe('RobotTable', () => {
  it('shows all robots', () => {
    const robots = [makeRobot({ name: 'test_robot1' }), makeRobot({ name: 'test_robot2' })];
    const tableData: RobotTableData[] = robots.map((robot) => ({
      name: robot.name,
    }));
    const root = render(<RobotTable robots={tableData} />);
    expect(root.getByText('test_robot1')).toBeTruthy();
    expect(root.getByText('test_robot2')).toBeTruthy();
  });

  it('smoke test for different robot status', () => {
    const robots = allStatuses.map((status) => makeRobot({ name: `${status}_robot`, status }));
    render(<RobotTable robots={robots} />);
  });

  it('pagination is shown when pagination option is provided', () => {
    const spy = jasmine.createSpy();
    const root = render(
      <RobotTable
        robots={[makeRobot()]}
        paginationOptions={{
          count: 1,
          page: 0,
          rowsPerPage: 10,
          rowsPerPageOptions: [10],
          onPageChange: spy,
        }}
      />,
    );
    // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    expect(root.getByText('1–1 of 1')).toBeTruthy();
  });

  it('pagination is not shown when no pagination option is provided', () => {
    const root = render(<RobotTable robots={[makeRobot()]} />);
    // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    expect(root.queryByText('1–1 of 1')).toBeNull();
  });
});
