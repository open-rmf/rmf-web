import React from 'react';
import { render } from '@testing-library/react';
import { DoorTableData, DoorDataGridTable } from './door-table-datagrid';
import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';
import { makeDoorState } from './test-utils.spec';

describe('DoorDataGridTable', () => {
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
      opMode: '',
    },
  ];

  it('renders doors data correctly', () => {
    const root = render(<DoorDataGridTable doors={mockDoors} />);
    const doorName = root.getByText('L3_door2');
    const levelName = root.getByText('L3');
    const doorType = root.getByText('Single Swing');
    const doorState = root.getByText('MOVING');

    expect(doorName).toBeTruthy();
    expect(levelName).toBeTruthy();
    expect(doorType).toBeTruthy();
    expect(doorState).toBeTruthy();
  });

  it('shows the correct number of rows', () => {
    const root = render(<DoorDataGridTable doors={mockDoors} />);
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(1);
  });

  it('shows titles correctly', () => {
    const root = render(<DoorDataGridTable doors={mockDoors} />);
    expect(root.queryByText('Name')).toBeTruthy();
    expect(root.queryByText('Current Floor')).toBeTruthy();
    expect(root.queryByText('Type')).toBeTruthy();
    expect(root.queryByText('Door State')).toBeTruthy();
  });
});
