import type { Dispenser, DispenserState, Ingestor, IngestorState } from 'api-client';

export type Workcell = Dispenser | Ingestor;
export type WorkcellState = DispenserState | IngestorState;
