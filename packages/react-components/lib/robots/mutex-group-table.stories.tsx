import React from 'react';
import { storiesOf } from '@storybook/react';
import { MutexGroupData, MutexGroupTable } from './mutex-group-table';

// Define the stories
storiesOf('Components/MutexGroupTable', module).add('Default', () => {
  const mutexGroups: MutexGroupData[] = [
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
  ];

  return <MutexGroupTable mutexGroups={mutexGroups} />;
});
