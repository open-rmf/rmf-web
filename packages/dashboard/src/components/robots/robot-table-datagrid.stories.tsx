import { Meta } from '@storybook/react';
import { ApiServerModelsRmfApiRobotStateStatus as RobotStatus } from 'api-client';

import { RobotDataGridTable } from './robot-table-datagrid';

export default {
  title: 'RobotDataGridTable',
} satisfies Meta;

export function Default() {
  return (
    <RobotDataGridTable
      robots={[
        {
          fleet: 'Fleet A',
          name: 'Robot 1',
          estFinishTime: new Date('2023-05-01T09:00:00').getTime(),
          battery: 0.8,
          lastUpdateTime: new Date('2023-05-01T08:30:00').getTime(),
          status: RobotStatus.Working,
        },
        {
          fleet: 'Fleet B',
          name: 'Robot 2',
          estFinishTime: new Date('2023-05-01T10:30:00').getTime(),
          battery: 0.6,
          lastUpdateTime: new Date('2023-05-01T10:00:00').getTime(),
          status: RobotStatus.Charging,
        },
        {
          fleet: 'Fleet A',
          name: 'Robot 3',
          estFinishTime: new Date('2023-05-01T11:45:00').getTime(),
          battery: 0.9,
          lastUpdateTime: new Date('2023-05-01T11:30:00').getTime(),
          status: RobotStatus.Idle,
        },
      ]}
    />
  );
}
