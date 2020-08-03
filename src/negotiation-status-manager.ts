import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';
import { NegotiationStatusTable, negotiationStatusConclusion } from '@osrf/romi-js-core-interfaces';

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

        if (negotiationStatusConclusion.resolved)
          conflict.resolved = ResolveState.RESOLVED;
        else
          conflict.resolved = ResolveState.NOT_RESOLVED;
        
        for (var [participant_id, status] of Object.entries(conflict.participant_ids_to_status))
        {
          if (negotiationStatusConclusion.resolved)
            status.status = NegotiationStatusTable.STATUS_FINISHED;
          else
            status.status = NegotiationStatusTable.STATUS_DEFUNCT;
        }
      });
  }

  private _conflicts : Record<string, NegotiationConflict> = {};
}