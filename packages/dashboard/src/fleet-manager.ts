import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class FleetManager extends EventEmitter<Events> {
  fleets(): RomiCore.FleetState[] {
    return Array.from(Object.values(this._fleets));
  }

  cachedFleets(): RomiCore.FleetState[] {
    return [...this._fleetCache];
  }

  startSubscription(transport: RomiCore.Transport) {
    this._subscriptions.push(
      transport.subscribe(RomiCore.fleetStates, (fleetState) => {
        this._updateCache(fleetState);
        // this assignment is for testing purposes
        // remember to remove it before mergin
        fleetState.robots = [
          {
            battery_percent: 0,
            location: {
              t: { sec: 0, nanosec: 0 },
              x: 11.553829193115234,
              y: -11.316267013549805,
              yaw: -1.6157349348068237,
              level_name: 'L1',
            },
            mode: { mode: 2 },
            model: '',
            name: 'tinyRobot1',
            path: [],
            task_id: '',
          },
        ];
        this._fleets[fleetState.name] = fleetState;
        this.emit('updated');
      }),
    );
  }

  stopAllSubscriptions(): void {
    this._subscriptions.forEach((sub) => sub.unsubscribe());
  }

  private _updateCache(fleetState: RomiCore.FleetState): void {
    if (
      this._fleetCache.length === 0 ||
      !this._fleetCache.some((fleet) => fleet.name === fleetState.name)
    ) {
      this._fleetCache.push(fleetState);
    } else if (this._fleetCache.some((fleet) => fleet.name === fleetState.name)) {
      this._fleetCache.forEach((fleet) => {
        if (fleet.name === fleetState.name) {
          this._addNewRobots(fleetState.robots, fleet.robots);
        }
      });
    }
  }

  private _addNewRobots(
    fleetStateRobots: RomiCore.RobotState[],
    cacheRobots: RomiCore.RobotState[],
  ): void {
    fleetStateRobots.forEach((fleetRobot) => {
      if (!cacheRobots.some((robot) => robot.name === fleetRobot.name)) {
        cacheRobots.push(fleetRobot);
      }
    });
  }

  private _fleets: Record<string, RomiCore.FleetState> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
  private _fleetCache: RomiCore.FleetState[] = [];
}
