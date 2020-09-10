import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export enum NegotiationState {
  NOT_RESOLVED = 0,
  RESOLVED,
}

export enum ResolveState {
  UNKNOWN = 0,
  RESOLVED,
  FAILED,
}

export class NegotiationStatus {
  sequence: number[] = [];
  defunct: boolean = false;
  rejected: boolean = false;
  forfeited: boolean = false;
}

export class NegotiationStatusData {
  hasTerminal: boolean = false;
  base: NegotiationStatus = new NegotiationStatus();
  terminal: NegotiationStatus = new NegotiationStatus();
}

export class NegotiationConflict {
  public participantIdsToNames: Record<string, string> = {};
  public participantIdsToStatus: Record<string, NegotiationStatusData> = {};
  public resolved: ResolveState = ResolveState.UNKNOWN;
}

export default class NegotiationStatusManager extends EventEmitter<Events> {
  constructor(url: string) {
    super();
    if (url) this._backendWs = new WebSocket(url);
  }

  allConflicts(): Record<number, NegotiationConflict> {
    return this._conflicts;
  }

  startSubscription() {
    if (!this._backendWs) {
      console.warn('backend websocket not available');
      return;
    }
    this._backendWs.send(JSON.stringify({ request: 'negotiation_update_subscribe' }));

    this._backendWs.onmessage = event => {
      const msg = JSON.parse(event.data);
      if (msg['type'] === 'negotiation_status') {
        const conflictVersion: number = msg['conflict_version'];
        const conflictVersionStr = conflictVersion.toString();

        let conflict = this._conflicts[conflictVersionStr];
        if (conflict === undefined) {
          conflict = new NegotiationConflict();

          const participantId: number = msg['participant_id'];
          conflict.participantIdsToNames[participantId] = msg['participant_name'];

          this._conflicts[conflictVersionStr] = conflict;
        }

        const id: number = msg['participant_id'];
        const idStr = id.toString();
        conflict.participantIdsToNames[idStr] = msg['participant_name'];

        let statusData = conflict.participantIdsToStatus[idStr];
        let status: NegotiationStatus;
        if (statusData === undefined) {
          statusData = new NegotiationStatusData();
          conflict.participantIdsToStatus[idStr] = statusData;
          status = statusData.base;
        } else {
          const seq: number[] = msg['sequence'];
          if (seq.length === 1) status = statusData.base;
          else {
            statusData.hasTerminal = true;
            status = statusData.terminal;
          }
        }

        status.defunct = msg['defunct'];
        status.rejected = msg['rejected'];
        status.forfeited = msg['forfeited'];
        status.sequence = msg['sequence'];

        this.emit('updated');
      } else if (msg['type'] === 'negotiation_conclusion') {
        const conflictVersion: number = msg['conflict_version'];
        const conflict = this._conflicts[conflictVersion.toString()];

        if (conflict === undefined) {
          console.warn('Undefined conflict version ' + conflictVersion + ', ignoring...');
          return;
        }

        if (msg['resolved'] === true) conflict.resolved = ResolveState.RESOLVED;
        else conflict.resolved = ResolveState.FAILED;

        this.removeOldConflicts();
      }
    };
  }

  removeOldConflicts() {
    const retainCount = 50;
    let resolved: string[] = [];

    for (const [version, status] of Object.entries(this._conflicts)) {
      if (status.resolved & ResolveState.RESOLVED) resolved.push(version);
    }
    resolved.sort(); // ascending

    // pop from the front until you reach the desired retain count
    while (resolved.length !== 0 && resolved.length > retainCount) {
      const key = resolved[0];
      console.log('removing resolved conflict: ' + key);
      delete this._conflicts[key];
      resolved.splice(0, 1);
    }
  }

  private _conflicts: Record<string, NegotiationConflict> = {};
  private _backendWs?: WebSocket;
}
