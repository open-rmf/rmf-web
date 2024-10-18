import { Meta, StoryObj } from '@storybook/react';
import { LiftState as RmfLiftState } from 'rmf-models/ros/rmf_lift_msgs/msg';

import { LiftDataGridTable, LiftTableData } from './lift-table-datagrid';
import { makeLift } from './test-utils.test';

const lift = makeLift();
const mockLifts: LiftTableData[] = [
  {
    index: 1,
    name: 'Lift1',
    currentFloor: 'L1',
    destinationFloor: 'L2',
    doorState: RmfLiftState.DOOR_OPEN,
    motionState: RmfLiftState.MOTION_DOWN,
    lift: lift,
  },
];

const meta: Meta<typeof LiftDataGridTable> = {
  title: 'LiftDataGridTable',
  component: LiftDataGridTable,
  decorators: [
    (Story) => (
      <div style={{ height: 800 }}>
        <Story />
      </div>
    ),
  ],
};

export default meta;

type Story = StoryObj<typeof LiftDataGridTable>;

export const Default: Story = {
  args: {
    lifts: mockLifts,
  },
};

export const Empty: Story = {
  args: {
    lifts: [],
  },
};
