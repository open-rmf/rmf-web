import * as RmfModels from 'rmf-models';

interface Dispenser {
  guid: string;
}

export interface DispenserPanelProps {
  dispensers: Dispenser[];
  dispenserStates: Record<string, RmfModels.DispenserState>;
}

export interface DispenserCellProps {
  dispenser: Dispenser;
  dispenserState: RmfModels.DispenserState;
}

export interface DispenserTableProps extends DispenserPanelProps {}
export interface DispenserRowProps extends DispenserCellProps {}

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
