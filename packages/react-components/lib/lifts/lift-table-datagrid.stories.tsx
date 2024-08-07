import { LiftState as RmfLiftState } from 'rmf-models/ros/rmf_lift_msgs/msg';

import { LiftDataGridTable, LiftTableData } from './lift-table-datagrid';
import { makeLift } from './test-utils.spec';

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

export default {
  title: 'LiftDataGridTable',
};

export const Default = () => <LiftDataGridTable lifts={mockLifts} />;
export const Empty = () => <LiftDataGridTable lifts={[]} />;
