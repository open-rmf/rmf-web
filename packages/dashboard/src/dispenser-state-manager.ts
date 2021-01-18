import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class DispenserStateManager extends EventEmitter<Events> {
  dispenserStates(): Record<string, RomiCore.DispenserState> {
    return { ...this._dispenserStates };
  }

  startSubscription(transport: RomiCore.Transport) {
    this._subscriptions.push(
      transport.subscribe(RomiCore.dispenserStates, (dispenserState) => {
        this.count += 1;
        if (this.count > 5 && this.count < 10) {
          this._dispenserStates['coke_ingestor'] = {
            guid: 'coke_ingestor',
            time: { sec: 0, nanosec: 0 },
            mode: 1,
            request_guid_queue: [],
            seconds_remaining: 0,
          };
        }
        if (this.count === 10) {
          this._dispenserStates = {};
          this._dispenserStates[dispenserState.guid] = dispenserState;
        }
        this._dispenserStates[dispenserState.guid] = dispenserState;
        this.emit('updated');
      }),
    );
  }
  stopAllSubscriptions(): void {
    this._subscriptions.forEach((sub) => sub.unsubscribe());
  }

  private _dispenserStates: Record<string, RomiCore.DispenserState> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
  count = 0;
}
