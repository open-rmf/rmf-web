import { Meta, StoryObj } from '@storybook/react';
import { Door as RmfDoor } from 'rmf-models/ros/rmf_building_map_msgs/msg';
import { DoorMode as RmfDoorMode } from 'rmf-models/ros/rmf_door_msgs/msg';

import { DoorDataGridTable, DoorTableData } from './door-table-datagrid';
import { makeDoorState } from './test-utils.test';

const mockDoors: DoorTableData[] = [
  {
    index: 1,
    doorName: 'L3_door2',
    levelName: 'L3',
    doorType: RmfDoor.DOOR_TYPE_SINGLE_SWING,
    doorState: makeDoorState({
      door_name: 'L3_door2',
      current_mode: { value: RmfDoorMode.MODE_MOVING },
    }),
  },
];

const meta: Meta<typeof DoorDataGridTable> = {
  title: 'DoorDataGridTable',
  component: DoorDataGridTable,
  decorators: [
    (Story) => (
      <div style={{ height: 800 }}>
        <Story />
      </div>
    ),
  ],
};

export default meta;

type Story = StoryObj<typeof DoorDataGridTable>;

export const Default: Story = {
  args: {
    doors: mockDoors,
  },
};

export const Empty: Story = {
  args: {
    doors: [],
  },
};
