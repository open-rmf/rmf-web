import { Meta, Story } from '@storybook/react';
import React from 'react';
import { PaginationOptions, RobotTable, RobotTableProps } from './robot-table';
import { RobotTableData } from './robot-table';
import { Status2 as RobotStatus } from 'api-client';

export default {
  title: 'Robots/Table',
} as Meta;

export const Table: Story<RobotTableProps> = (args) => {
  const [page, setPage] = React.useState(0);
  const paginationOptions: PaginationOptions = {
    count: args.robots.length,
    rowsPerPage: 10,
    rowsPerPageOptions: [10],
    page,
    onPageChange: (_ev, newPage) => setPage(newPage),
  };

  return (
    <>
      <RobotTable
        {...args}
        style={{ height: '95vh', display: 'flex', flexDirection: 'column' }}
        robots={robots.slice(page * 10, (page + 1) * 10)}
        paginationOptions={paginationOptions}
      />
    </>
  );
};

const allStatuses: RobotStatus[] = Object.values(RobotStatus) as RobotStatus[];

const robots: RobotTableData[] = [];
for (let i = 0; i < 12; ++i) {
  robots.push({
    name: `Robot ${i + 1}`,
    battery: Math.min(i / 10, 1),
    status: allStatuses[i % allStatuses.length],
    estFinishTime: Date.now() + i * 1000000,
    lastUpdateTime: Date.now(),
  });
}

Table.args = {
  robots,
};
