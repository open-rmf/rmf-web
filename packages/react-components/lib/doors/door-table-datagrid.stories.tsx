import React from 'react';
import { DoorTableData, DoorDataGridTable } from './door-table-datagrid';
import { storiesOf } from '@storybook/react';
import { action } from '@storybook/addon-actions';
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
    onClickOpen: action('onClickOpen'),
    onClickClose: action('onClickClose'),
  },
];

storiesOf('DoorDataGridTable', module)
  .add('Default', () => <DoorDataGridTable doors={mockDoors} />)
  .add('Empty', () => <DoorDataGridTable doors={[]} />);
