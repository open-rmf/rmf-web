import React from 'react';
import { render } from '@testing-library/react';
import { LiftDataGridTable, LiftTableData } from './lift-table-datagrid';
import { LiftState as RmfLiftState } from 'rmf-models';
import { makeLift } from './test-utils.spec';

describe('LiftDataGridTable', () => {
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
      opMode: '',
    },
  ];

  it('renders lift data correctly', () => {
    const root = render(<LiftDataGridTable lifts={mockLifts} />);

    const liftName = root.getByText('Lift1');
    const currentFloor = root.getByText('L1');
    const destinationFloor = root.getByText('L2');
    const doorState = root.getByText('OPEN');

    expect(liftName).toBeTruthy();
    expect(currentFloor).toBeTruthy();
    expect(destinationFloor).toBeTruthy();
    expect(doorState).toBeTruthy();
  });

  it('shows the correct number of rows', () => {
    const root = render(<LiftDataGridTable lifts={mockLifts} />);
    const allRows = root.container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(1);
  });

  it('shows titles correctly', () => {
    const root = render(<LiftDataGridTable lifts={mockLifts} />);
    expect(root.queryByText('Name')).toBeTruthy();
    expect(root.queryByText('Current Floor')).toBeTruthy();
    expect(root.queryByText('Destination Floor')).toBeTruthy();
    expect(root.queryByText('Lift State')).toBeTruthy();
  });
});
