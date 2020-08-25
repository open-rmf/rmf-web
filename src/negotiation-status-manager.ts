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
  has_terminal : boolean = false;
  base : NegotiationStatus = new NegotiationStatus();
  terminal : NegotiationStatus = new NegotiationStatus();
}

export class NegotiationConflict {
  public participant_ids_to_names : Record<string, string> = {};
  public participant_ids_to_status : Record<string, NegotiationStatusData> = {};
  public resolved : ResolveState = ResolveState.UNKNOWN;
};

export default class NegotiationStatusManager extends EventEmitter<Events> {
  constructor(private _backend_ws ?: WebSocket) {
    super();
  }

  updateNegotiationStatus(msg : any) {
    var conflict_version : number = msg["conflict_version"];
    var conflict_version_str = conflict_version.toString()

    var conflict = this._conflicts[conflict_version_str];
    if (conflict === undefined)
    {
      conflict = new NegotiationConflict();

      var participant_id : number = msg["participant_id"]      
      conflict.participant_ids_to_names[participant_id] = msg["participant_name"];

      this._conflicts[conflict_version_str] = conflict;
    }

    var id : number = msg["participant_id"];
    var id_str = id.toString();
    conflict.participant_ids_to_names[id_str] = msg["participant_name"];

    var status_data = conflict.participant_ids_to_status[id_str];
    var status : NegotiationStatus;
    if (status_data === undefined)
    {
      status_data = new NegotiationStatusData();
      conflict.participant_ids_to_status[id_str] = status_data;
      status = status_data.base; 
    }
    else
    {
      var seq : number[] = msg["sequence"];
      if (seq.length === 1)
        status = status_data.base;
      else
      {
        status_data.has_terminal = true;
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
    var conflict_version : number = msg["conflict_version"];
    var conflict = this._conflicts[conflict_version.toString()];

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
    var negotiation_status_mgr = this;
    if (this._backend_ws) {
      //only recv negotiation status messages
      this._backend_ws.onmessage = function (event) {
        var msg = JSON.parse(event.data);
        if (msg["type"] === "negotiation_status")
        {
          negotiation_status_mgr.updateNegotiationStatus(msg);
        }
        else if (msg["type"] === "negotiation_conclusion")
        {
          negotiation_status_mgr.concludeNegotiationStatus(msg);
        }
      };
    }
  }

  removeOldConflicts() {
    var retain_count = 99; // increase this value for testing
    var resolved : string[] = [];

    for (var [version, status] of Object.entries(this._conflicts))
    {
      if (status.resolved & ResolveState.RESOLVED)
        resolved.push(version);
    }
    resolved.sort(); //ascending
    
    // pop from the front until you reach the desired retain count
    while (resolved.length !== 0 && resolved.length > retain_count)
    {
      var key = resolved[0];
      console.log("removing resolved conflict: " + key);
      delete this._conflicts[key];
      resolved.splice(0, 1);
    }
  }

  private _conflicts : Record<string, NegotiationConflict> = {};
}