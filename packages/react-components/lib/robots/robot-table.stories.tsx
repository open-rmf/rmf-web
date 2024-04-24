import { Meta, StoryFn } from '@storybook/react';
import { ApiServerModelsRmfApiRobotStateStatus as RobotStatus } from 'api-client';
import React from 'react';
import { RobotTable, RobotTableData, RobotTableProps } from './robot-table';

export default {
  title: 'Robots/Table',
} satisfies Meta;

export const Table: StoryFn<RobotTableProps> = (args) => {
  return (
    <>
      <RobotTable
        {...args}
        style={{ height: '95vh', display: 'flex', flexDirection: 'column' }}
        robots={robots}
      />
    </>
  );
};

const allStatuses: RobotStatus[] = Object.values(RobotStatus) as RobotStatus[];

const robots: RobotTableData[] = [];
for (let i = 0; i < 12; ++i) {
  robots.push({
    fleet: 'test',
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
