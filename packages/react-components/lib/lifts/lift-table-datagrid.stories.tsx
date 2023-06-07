import React from 'react';
import { storiesOf } from '@storybook/react';
import { action } from '@storybook/addon-actions';
import { LiftDataGridTable, LiftTableData } from './lift-table-datagrid';
import { makeLift } from './test-utils.spec';
import { LiftState as RmfLiftState } from 'rmf-models';

const lift = makeLift();
const mockLifts: LiftTableData[] = [
  {
    index: 1,
    name: 'Lift1',
    current_floor: 'L1',
    destination_floor: 'L2',
    door_state: RmfLiftState.DOOR_OPEN,
    motion_state: RmfLiftState.MOTION_DOWN,
    lift: lift,
    onRequestSubmit: action('onRequestSubmit'),
  },
];

storiesOf('LiftDataGridTable', module)
  .add('Default', () => <LiftDataGridTable lifts={mockLifts} />)
  .add('Empty', () => <LiftDataGridTable lifts={[]} />);
