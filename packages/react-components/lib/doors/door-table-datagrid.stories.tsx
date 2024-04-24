import React from 'react';
import { DoorTableData, DoorDataGridTable } from './door-table-datagrid';
import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';
import { makeDoorState } from './test-utils.spec';

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
    opMode: 'test',
  },
];

export default {
  title: 'DoorDataGridTable',
};

export const Default = () => <DoorDataGridTable doors={mockDoors} />;
export const Empty = () => <DoorDataGridTable doors={[]} />;
