import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class FleetManager extends EventEmitter<Events> {
  fleets(): Record<string, RomiCore.RobotState[]> {
    return { ...this._fleets };
  }

  cachedFleets(): Record<string, RomiCore.RobotState[]> {
    return { ...this._robotCache };
  }

  startSubscription(transport: RomiCore.Transport) {
    this._subscriptions.push(
      transport.subscribe(RomiCore.fleetStates, (fleetState) => {
        this.counter += 1;

        // remember to remove after testing
        if (this.counter < 20 || this.counter > 40) {
          fleetState = { name: 'tinyRobot', robots: [] };
          //   fleetState.robots = [
          //     {
          //       battery_percent: 0,
          //       location: {
          //         t: { sec: 0, nanosec: 0 },
          //         x: 11.553829193115234,
          //         y: -11.316267013549805,
          //         yaw: -1.6157349348068237,
          //         level_name: 'L1',
          //       },
          //       mode: { mode: 2 },
          //       model: '',
          //       name: 'tinyRobot1',
          //       path: [],
          //       task_id: '',
          //     },
          //   ];
        }

        this._fleets[fleetState.name] = fleetState.robots;
        this.updateCache(fleetState);
        this.emit('updated');
      }),
    );
  }

  stopAllSubscriptions(): void {
    this._subscriptions.forEach((sub) => sub.unsubscribe());
  }

  private updateCache(fleetState: RomiCore.FleetState) {
    if (!this._robotCache[fleetState.name]) {
      this._robotCache[fleetState.name] = fleetState.robots;
    } else {
      fleetState.robots.forEach((robot) => {
        if (
          !this._robotCache[fleetState.name].some((cacheRobot) => cacheRobot.name === robot.name)
        ) {
          this._robotCache[fleetState.name].push(robot);
        }
      });
    }
  }

  private _fleets: Record<string, RomiCore.RobotState[]> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
  private _robotCache: Record<string, RomiCore.RobotState[]> = {};
  counter: number = 1;
}
