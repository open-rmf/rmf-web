import * as RmfModels from 'rmf-models';

export function dispenserModeToString(mode: number): string {
  switch (mode) {
    case RmfModels.DispenserState.IDLE:
      return 'IDLE';
    case RmfModels.DispenserState.BUSY:
      return 'ONLINE';
    case RmfModels.DispenserState.OFFLINE:
      return 'OFFLINE';
    default:
      return 'N/A';
  }
}

// table cell has padding of 16px left and 24px right respectively
// need to deduct 40px away from actual width
export const workCellTableCellConfig = {
  dispenserName: 113.6,
  opMode: 68,
  numQueueRequest: 148,
  requestQueueId: 123.2,
  // last column deduct 32px
  secRemaining: 136.8,
  rowHeight: 31,
};
