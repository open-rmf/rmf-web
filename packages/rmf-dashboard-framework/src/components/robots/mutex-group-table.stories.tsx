import { Meta, StoryObj } from '@storybook/react';

import { MutexGroupTable } from './mutex-group-table';

const meta: Meta<typeof MutexGroupTable> = {
  title: 'Robots/MutexGroupTable',
  component: MutexGroupTable,
  decorators: [
    (Story) => (
      <div style={{ height: 800 }}>
        <Story />
      </div>
    ),
  ],
};

export default meta;

type Story = StoryObj<typeof MutexGroupTable>;

export const Default: Story = {
  args: {
    mutexGroups: [
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
    ],
  },
};
