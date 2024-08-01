import { Meta } from '@storybook/react';

import { MutexGroupTable } from './mutex-group-table';

export default {
  title: 'MutexGroupTable',
} satisfies Meta;

export function Default() {
  <MutexGroupTable
    mutexGroups={[
      {
        name: 'group1',
        lockedBy: 'fleet1/robot1',
        requestedBy: ['fleet1/robot2', 'fleet2/robot1'],
      },
      {
        name: 'group2',
        lockedBy: 'fleet1/robot1',
        requestedBy: ['fleet1/robot2'],
      },
    ]}
  />;
}
