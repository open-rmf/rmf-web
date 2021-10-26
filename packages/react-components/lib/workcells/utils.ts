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
  dispenserName: 0.196,
  opMode: 0.138,
  numQueueRequest: 0.241,
  requestQueueId: 0.208,
  // last column deduct 32px
  secRemaining: 0.217,
  rowHeight: 31,
};
