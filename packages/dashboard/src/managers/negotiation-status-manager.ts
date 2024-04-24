import { Trajectory } from './robot-trajectory-manager';
import TrajectorySocketManager from './trajectory-socket-manager';
import { Authenticator } from 'rmf-auth';

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

export interface NegotiationTrajectoryRequest {
  request: 'negotiation_trajectory';
  param: {
    conflict_version: number;
    sequence: number[];
  };
  token?: string;
}

export interface NegotiationSubscribeUpdateRequest {
  request: 'negotiation_update_subscribe';
  token?: string;
}

export interface NegotiationTrajectoryResponse {
  response: 'negotiation_trajectory';
  values: Trajectory[];
  error?: string;
}

export class NegotiationStatusManager extends TrajectorySocketManager {
  constructor(ws: WebSocket | undefined, authenticator?: Authenticator) {
    super();
    if (ws) this._webSocket = ws;
    if (authenticator) this._authenticator = authenticator;
  }

  allConflicts(): Record<number, NegotiationConflict> {
    return this._conflicts;
  }

  getLastUpdateTS(): number {
    return this._statusUpdateLastTS;
  }

  startSubscription(token?: string): void {
    if (!this._webSocket) {
      console.warn('backend websocket not available');
      return;
    }
    if (this._webSocket.readyState === WebSocket.OPEN) {
      const negotiationUpdate: NegotiationSubscribeUpdateRequest = {
        request: 'negotiation_update_subscribe',
        token: token,
      };
      this._webSocket.send(JSON.stringify(negotiationUpdate));

      this._webSocket.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg['type'] === 'negotiation_status') {
          this._statusUpdateLastTS = Date.now();

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
          this._statusUpdateLastTS = Date.now();

          const conflictVersion: number = msg['conflict_version'];
          const conflict = this._conflicts[conflictVersion.toString()];

          if (conflict === undefined) {
            console.warn('Undefined conflict version ' + conflictVersion + ', ignoring...');
            return;
          }

          if (msg['resolved'] === true) conflict.resolved = ResolveState.RESOLVED;
          else conflict.resolved = ResolveState.FAILED;

          this.removeOldConflicts();
        } else if (msg.error) {
          if (msg.error === 'token expired') this._authenticator?.logout();
          else {
            throw new Error(msg.error);
          }
        }
      };
    } else {
      this._webSocket.onopen = () => this.startSubscription(token);
    }
  }

  removeOldConflicts(): void {
    const retainCount = 50;
    const resolved: string[] = [];

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

  async negotiationTrajectory(
    request: NegotiationTrajectoryRequest,
  ): Promise<NegotiationTrajectoryResponse> {
    await this._authenticator?.refreshToken();
    const event = await this.send(JSON.stringify(request), this._webSocket);
    const resp = JSON.parse(event.data);

    if (resp.values === null) {
      resp.values = [];
    }
    return resp as NegotiationTrajectoryResponse;
  }

  private _conflicts: Record<string, NegotiationConflict> = {};
  private _webSocket?: WebSocket;
  private _statusUpdateLastTS: number = -1;
  private _authenticator?: Authenticator;
}
