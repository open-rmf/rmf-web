import type { Dispenser, DispenserState, Ingestor, IngestorState } from 'api-client';

export * from './workcell-panel';
export * from './workcell-table';

export type Workcell = Dispenser | Ingestor;
export type WorkcellState = DispenserState | IngestorState;
