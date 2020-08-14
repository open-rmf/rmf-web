import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';
import { NegotiationStatus, negotiationStatusConclusion } from '@osrf/romi-js-core-interfaces';

type Events = {
  updated: [];
};

export enum ResolveState {
  UNKNOWN = 0,
  RESOLVED,
  NOT_RESOLVED
};

export class NegotiationConflict {
  public participant_ids_to_names : Record<string, string> = {};
  public participant_ids_to_status : Record<string, RomiCore.NegotiationStatus> = {};
  public resolved : number = ResolveState.UNKNOWN;
}

export default class NegotiationStatusManager extends EventEmitter<Events> {
  allConflicts(): Record<number, NegotiationConflict> {
    return this._conflicts;
  }

  startSubscription(transport: RomiCore.Transport) {
    transport.subscribe(RomiCore.skipValidation(RomiCore.negotiationStatus), negotiationStatus => {
      
      var conflict = this._conflicts[negotiationStatus.conflict_version.toString()];
      if (conflict === undefined)
      {
        this._conflicts[negotiationStatus.conflict_version.toString()] = new NegotiationConflict();
        conflict = this._conflicts[negotiationStatus.conflict_version.toString()];
      }
      var id = negotiationStatus.participant.toString();
      
      conflict.participant_ids_to_names[id] = negotiationStatus.participant_name;
      conflict.participant_ids_to_status[id] = negotiationStatus;
      this.emit('updated');
    });

    transport.subscribe(RomiCore.skipValidation(RomiCore.negotiationStatusConclusion), 
      negotiationStatusConclusion => {
        var version = negotiationStatusConclusion.conflict_version.toString();
        var conflict = this._conflicts[version];

        if (conflict === undefined)
        {
          console.warn('Undefined conflict version ' + version + ', ignoring...');
          return;
        }

        if (negotiationStatusConclusion.resolved)
          conflict.resolved = ResolveState.RESOLVED;
        else
          conflict.resolved = ResolveState.NOT_RESOLVED;
        
        for (var [participant_id, status] of Object.entries(conflict.participant_ids_to_status))
        {
          if (negotiationStatusConclusion.resolved)
            status.status = NegotiationStatus.STATUS_FINISHED;
          else
            status.status = NegotiationStatus.STATUS_DEFUNCT;
        }
        
        this.removeOldConflicts();
      });
  }

  removeOldConflicts() {
    var retain_count = 0; // increase this value for testing
    var resolved : string[] = [];

    for (var [version, status] of Object.entries(this._conflicts))
    {
      if (status.resolved & NegotiationStatus.STATUS_FINISHED)
        resolved.push(version);
    }
    resolved.sort(); //ascending
    
    // pop from the front until you reach the desired retain count
    while (resolved.length != 0 && resolved.length > retain_count)
    {
      var key = resolved[0];
      console.log("removing resolved conflict: " + key);
      delete this._conflicts[key];
      resolved.splice(0, 1);
    }
  }

  private _conflicts : Record<string, NegotiationConflict> = {};
}