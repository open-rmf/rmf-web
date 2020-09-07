import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export enum NegotiationState {
  NOT_RESOLVED = 0,
  RESOLVED
};

export enum ResolveState {
  UNKNOWN = 0,
  RESOLVED,
  FAILED
};


export class NegotiationStatus {
  sequence : number[] = [];
  defunct : boolean = false;
  rejected : boolean = false;
  forfeited : boolean = false;
};

export class NegotiationStatusData {
  hasTerminal : boolean = false;
  base : NegotiationStatus = new NegotiationStatus();
  terminal : NegotiationStatus = new NegotiationStatus();
}

export class NegotiationConflict {
  public participantIdsToNames : Record<string, string> = {};
  public participantIdsToStatus : Record<string, NegotiationStatusData> = {};
  public resolved : ResolveState = ResolveState.UNKNOWN;
};

export default class NegotiationStatusManager extends EventEmitter<Events> {
  constructor(private _backend_ws ?: WebSocket) {
    super();
  }

  updateNegotiationStatus(msg : any) {
    const conflict_version : number = msg["conflict_version"];
    const conflict_version_str = conflict_version.toString()

    let conflict = this._conflicts[conflict_version_str];
    if (conflict === undefined)
    {
      conflict = new NegotiationConflict();

      const participant_id : number = msg["participant_id"]      
      conflict.participantIdsToNames[participant_id] = msg["participant_name"];

      this._conflicts[conflict_version_str] = conflict;
    }

    const id : number = msg["participant_id"];
    const id_str = id.toString();
    conflict.participantIdsToNames[id_str] = msg["participant_name"];

    let status_data = conflict.participantIdsToStatus[id_str];
    let status : NegotiationStatus;
    if (status_data === undefined)
    {
      status_data = new NegotiationStatusData();
      conflict.participantIdsToStatus[id_str] = status_data;
      status = status_data.base; 
    }
    else
    {
      const seq : number[] = msg["sequence"];
      if (seq.length === 1)
        status = status_data.base;
      else
      {
        status_data.hasTerminal = true;
        status = status_data.terminal;
      }
    }

    status.defunct = msg["defunct"];
    status.rejected = msg["rejected"];
    status.forfeited = msg["forfeited"];
    status.sequence = msg["sequence"];

    this.emit('updated');
  }

  concludeNegotiationStatus(msg : any) {
    const conflict_version : number = msg["conflict_version"];
    const conflict = this._conflicts[conflict_version.toString()];

    if (conflict === undefined)
    {
      console.warn('Undefined conflict version ' + conflict_version + ', ignoring...');
      return;
    }

    if (msg["resolved"] === true)
      conflict.resolved = ResolveState.RESOLVED;
    else
      conflict.resolved = ResolveState.FAILED;
    
    this.removeOldConflicts();
  }

  allConflicts(): Record<number, NegotiationConflict> {
    return this._conflicts;
  }

  startSubscription() {
    if (this._backend_ws) {
      //only recv negotiation status messages
      this._backend_ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg["type"] === "negotiation_status")
        {
          this.updateNegotiationStatus(msg);
        }
        else if (msg["type"] === "negotiation_conclusion")
        {
          this.concludeNegotiationStatus(msg);
        }
      };
    }
  }

  removeOldConflicts() {
    const retain_count = 99; // increase this value for testing
    let resolved : string[] = [];

    for (const [version, status] of Object.entries(this._conflicts))
    {
      if (status.resolved & ResolveState.RESOLVED)
        resolved.push(version);
    }
    resolved.sort(); //ascending
    
    // pop from the front until you reach the desired retain count
    while (resolved.length !== 0 && resolved.length > retain_count)
    {
      const key = resolved[0];
      console.log("removing resolved conflict: " + key);
      delete this._conflicts[key];
      resolved.splice(0, 1);
    }
  }

  private _conflicts : Record<string, NegotiationConflict> = {};
}